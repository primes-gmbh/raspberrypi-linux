#include "linux/drbd.h"
#include "linux/fpga/fpga-mgr.h"
#include <linux/dma-buf.h>
#include <linux/kernel.h>
#include "linux/mod_devicetable.h"
#include <linux/mutex.h>
#include "linux/spinlock.h"
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/hw_breakpoint.h>
#include <linux/of.h>
#include <linux/iio/sysfs.h>
#include <linux/i2c.h>

#include <linux/fpga/fpga-region.h>

// #include "bcm2835-iio-dma-buffer.h"
#include "vc4-regs-unicam.h"

#include "dac.h"

#define UNICAM_MODULE_NAME "unicam"
#define UNICAM_VERSION "0.1.0"

#define BPL_ALIGNMENT 32
#define DUMMY_BUF_SIZE PAGE_SIZE

/*
 * Unicam must request a minimum of 250Mhz from the VPU clock.
 * Otherwise the input FIFOs overrun and cause image corruption.
 */
#define MIN_VPU_CLOCK_RATE (250 * 1000 * 1000)

enum {
	PRIMES_FPGA_I2C_ADDR_MIPI_LANES = 0xaa,
	PRIMES_FPGA_I2C_ADDR_MIPI_HACT = 0xac,
	PRIMES_FPGA_I2C_ADDR_MIPI_VACT = 0xbe,
};

/*
 * Size of the dummy buffer allocation.
 *
 * Due to a HW bug causing buffer overruns in circular buffer mode under certain
 * (not yet fully known) conditions, the dummy buffer allocation is set to a
 * a single page size, but the hardware gets programmed with a buffer size of 0.
 */

struct unicam_device {
	/* peripheral base address */
	void __iomem *base;
	/* clock gating base address */
	void __iomem *clk_gate_base;
	/* lp clock handle */
	struct clk *clock;
	/* vpu clock handle */
	struct clk *vpu_clock;
	/* clock status for error handling */
	bool clocks_enabled;

	/* parent device */
	struct platform_device *pdev;
	unsigned int sequence;

	/*
         * Stores bus.mipi_csi2.flags for CSI2 sensors, or
         * bus.mipi_csi1.strobe for CCP2.
         */
	unsigned int active_data_lanes;
	u16 hres;
	u16 vres;

	bool frame_started;

	struct mutex mx_streaming;
	bool streaming;

	size_t dummy_dma_size;
	dma_addr_t dummy_dma_addr;
	void *dummy_dma_vaddr;

	struct iio_dev *indio_dev;
	struct iio_dma_buffer_queue queue[1];
	struct iio_dma_buffer_block *cur_block, *next_block;
	spinlock_t list_lock;
	struct list_head block_list;

	u64 start_time;
	u64 total_frames;
	u64 frames_lost;
	bool dummy_scheduled;
	char *frame_lost_reason;

	struct i2c_client *fpga_i2c_mipi_config;
	struct i2c_client *fpga_i2c_amplifier_config;
	struct primes_dac dac0;
	struct primes_dac dac1;

	struct fpga_manager *fpga_mgr;
};

static int unicam_log_status(struct unicam_device *unicam);

/* Hardware access */
static inline void clk_write(struct unicam_device *dev, u32 val)
{
	writel(val | 0x5a000000, dev->clk_gate_base);
}

static inline u32 reg_read(struct unicam_device *dev, u32 offset)
{
	return readl(dev->base + offset);
}

static inline void reg_write(struct unicam_device *dev, u32 offset, u32 val)
{
	writel(val, dev->base + offset);
}

static inline int get_field(u32 value, u32 mask)
{
	return (value & mask) >> __ffs(mask);
}

static inline void set_field(u32 *valp, u32 field, u32 mask)
{
	u32 val = *valp;

	val &= ~mask;
	val |= (field << __ffs(mask)) & mask;
	*valp = val;
}

static inline u32 reg_read_field(struct unicam_device *dev, u32 offset,
				 u32 mask)
{
	return get_field(reg_read(dev, offset), mask);
}

static inline void reg_write_field(struct unicam_device *dev, u32 offset,
				   u32 field, u32 mask)
{
	u32 val = reg_read(dev, offset);

	set_field(&val, field, mask);
	reg_write(dev, offset, val);
}

/* Power management functions */
static inline int unicam_runtime_get(struct unicam_device *dev)
{
	return pm_runtime_get_sync(&dev->pdev->dev);
}

static inline void unicam_runtime_put(struct unicam_device *dev)
{
	pm_runtime_put_sync(&dev->pdev->dev);
}

static void unicam_wr_dma_addr(struct unicam_device *dev, dma_addr_t dmaaddr,
			       unsigned int buffer_size)
{
	dma_addr_t endaddr = dmaaddr + buffer_size;
	reg_write(dev, UNICAM_IBSA0, dmaaddr);
	reg_write(dev, UNICAM_IBEA0, endaddr);
}

static void unicam_schedule_next_block(struct unicam_device *unicam)
{
	struct iio_dma_buffer_block *block;
	dma_addr_t phys_addr;
	size_t size;
	int nents;
	struct scatterlist *sgl;

	block = list_first_entry(&unicam->block_list,
				 struct iio_dma_buffer_block, head);
	list_del(&block->head);
	phys_addr = block->phys_addr;
	size = block->size;
	nents = 1;
	if (block->sg_table) {
		sgl = block->sg_table->sgl;
		nents = sg_nents_for_len(sgl, block->bytes_used);
		if (nents != 1) {
			dev_err(&unicam->pdev->dev, "sg_table nents != 1\n");
			return;
		}
		phys_addr = sg_dma_address(sgl);
		size = sg_dma_len(sgl);
	}
	unicam_wr_dma_addr(unicam, phys_addr, size);
	unicam->next_block = block;
}

static void unicam_schedule_dummy_block(struct unicam_device *unicam)
{
	unicam_wr_dma_addr(unicam, unicam->dummy_dma_addr, 0);
	unicam->next_block = NULL;
}

static void unicam_process_block_done(struct unicam_device *unicam)
{
	iio_dma_buffer_block_done(unicam->cur_block);
}

/*
 * unicam_isr : ISR handler for unicam capture
 * @irq: irq number
 * @dev_id: dev_id ptr
 *
 * It changes status of the captured buffer, takes next buffer from the queue
 * and sets its address in unicam registers
 */

static irqreturn_t unicam_isr(int irq, void *dev)
{
	struct unicam_device *unicam = dev;
	u32 ista, sta;
	u32 ibwp, ibsa0, ibea0;
	bool fs, fe, lci;

	ibwp = reg_read(unicam, UNICAM_IBWP);
	sta = reg_read(unicam, UNICAM_STA);
	ista = reg_read(unicam, UNICAM_ISTA);

	if (sta & UNICAM_SBE) {
		dev_err(&unicam->pdev->dev, "short packet bit error");
	}
	if (sta & UNICAM_PBE) {
		dev_err(&unicam->pdev->dev, "payload bit error");
	}
	if (sta & UNICAM_HOE) {
		dev_err(&unicam->pdev->dev, "header overflow error");
	}
	if (sta & UNICAM_PLE) {
		dev_err(&unicam->pdev->dev, "payload overflow error");
	}
	if (sta & UNICAM_SSC) {
		dev_err(&unicam->pdev->dev, "start-of-frame sequence error");
	}
	if (sta & UNICAM_CRCE) {
		dev_err(&unicam->pdev->dev, "crc error");
	}
	if (sta & UNICAM_IFO) {
		dev_err(&unicam->pdev->dev, "input fifo overflow");
	}
	if (sta & UNICAM_OFO) {
		dev_err(&unicam->pdev->dev, "output fifo overflow");
	}
	if (sta & UNICAM_BFO) {
		dev_err(&unicam->pdev->dev, "byte fifo overflow");
	}
	if (sta & UNICAM_DL) {
		dev_err(&unicam->pdev->dev, "data lost");
	}
	if (sta & UNICAM_PS) {
		dev_err(&unicam->pdev->dev, "preamble short");
	}
	if (sta & UNICAM_FSI_S) {
		dev_err(&unicam->pdev->dev,
			"frame start interrupt status, sta: %08x ista: %08x frame_started: %d",
			sta, ista, unicam->frame_started);
	}
	if (sta & UNICAM_FEI_S) {
		dev_err(&unicam->pdev->dev, "frame end interrupt status");
	}
	if (sta & UNICAM_LCI_S) {
		dev_err(&unicam->pdev->dev, "line count interrupt status");
	}

	/* Write value back to clear the interrupts */
	reg_write(unicam, UNICAM_STA, sta);
	/* Write value back to clear the interrupts */
	reg_write(unicam, UNICAM_ISTA, ista);
	ibsa0 = reg_read(unicam, UNICAM_IBSA0);
	ibea0 = reg_read(unicam, UNICAM_IBEA0);

	if (!(sta & (UNICAM_IS | UNICAM_PI0))) {
		return IRQ_HANDLED;
	}

	/*
	 * Look for either the Frame End interrupt or the Packet Capture status
	 * to signal a frame end.
	 */
	fs = !!(ista & UNICAM_FSI);
	lci = !!(ista & UNICAM_LCI);
	fe = (ista & UNICAM_FEI || sta & UNICAM_PI0);

	/*
         * We must run the frame end handler first. If we have a valid next_frm
         * and we get a simultaneout FE + FS interrupt, running the FS handler
         * first would null out the next_frm ptr and we would have lost the
         * buffer forever.
         */
	if (fe && unicam->frame_started) {
		unicam->total_frames++;
		if (unicam->cur_block &&
		    unicam->cur_block != unicam->next_block) {
			unicam->cur_block->bytes_used = unicam->cur_block->size;
			unicam_process_block_done(unicam);
		}
		unicam->cur_block = unicam->next_block;
		unicam->frame_started = false;
	}
	if (fs) {
		if (unicam->dummy_scheduled) {
			// dev_warn(&unicam->pdev->dev,
			// 	 "frame lost - reason: %s\n",
			// 	 unicam->frame_lost_reason);
		}
		if (!unicam->next_block ||
		    unicam->cur_block == unicam->next_block) {
			unicam->dummy_scheduled = true;
			unicam->frame_lost_reason = "unknown";
			unicam_schedule_dummy_block(unicam);
		} else if (unicam->cur_block) {
			dev_warn(
				&unicam->pdev->dev,
				"frame lost fs: %d lci: %d fe: %d reason: %s\n",
				fs, lci, fe, unicam->frame_lost_reason);
			unicam->cur_block->bytes_used = 0;
			unicam_process_block_done(unicam);
			unicam->cur_block = unicam->next_block;
			unicam->next_block = NULL;
			unicam->frames_lost++;
		}
		unicam->frame_started = true;
	}

	// Schedule the next block
	if ((fs || lci) && !fe) {
		spin_lock(&unicam->list_lock);
		if (!list_empty(&unicam->block_list) && !unicam->next_block) {
			unicam_schedule_next_block(unicam);
			unicam->dummy_scheduled = false;
			unicam->frame_lost_reason = "unknown";
		} else if (list_empty(&unicam->block_list)) {
			unicam->frame_lost_reason = "block_list is empty";
		} else if (unicam->next_block) {
			unicam->frame_lost_reason = "next_block is not empty";
		}
		spin_unlock(&unicam->list_lock);
	} else {
		unicam->frame_lost_reason = "FSI and FEI at the same time";
	}
	return IRQ_HANDLED;
}

static void unicam_set_packing_config(struct unicam_device *dev)
{
	u32 pack, unpack;
	u32 val;

	unpack = UNICAM_PUM_UNPACK8;
	pack = UNICAM_PPM_PACK8;

	val = 0;
	set_field(&val, unpack, UNICAM_PUM_MASK);
	set_field(&val, pack, UNICAM_PPM_MASK);
	reg_write(dev, UNICAM_IPIPE, val);
}

static void unicam_cfg_image_id(struct unicam_device *dev, u32 csi_dt)
{
	reg_write(dev, UNICAM_IDI0, (0 << 6) | csi_dt);
}

static void unicam_start_rx(struct unicam_device *unicam)
{
	dev_info(&unicam->pdev->dev, "unicam_start_rx\n");

	unsigned int i;
	u32 val;

	/* It seems like line_int_freq should be set to something high enough to avoid certain linux
	 * errors such as CPU stalls or raspi frimware errors */
	int line_int_freq = unicam->vres >> 2;
	if (line_int_freq < 128) {
		line_int_freq = 128;
	}
	// int line_int_freq = 1;
	dev_info(
		&unicam->pdev->dev,
		"stride size: %d bytes, buffer_size: %zu bytes -> setting line_int_freq to %d\n",
		unicam->hres, unicam->queue[0].fileio.block_size,
		line_int_freq);

	/* Enable lane clocks */
	val = 1;
	for (i = 0; i < unicam->active_data_lanes; i++) {
		val = val << 2 | 1;
	}
	clk_write(unicam, val);

	/* Basic init */
	reg_write(unicam, UNICAM_CTRL, UNICAM_MEM);

	/* Enable analogue control, and leave in reset. */
	val = UNICAM_AR;
	set_field(&val, 7, UNICAM_CTATADJ_MASK);
	set_field(&val, 7, UNICAM_PTATADJ_MASK);
	reg_write(unicam, UNICAM_ANA, val);
	usleep_range(1000, 2000);

	/* Come out of reset */
	reg_write_field(unicam, UNICAM_ANA, 0, UNICAM_AR);

	/* Peripheral reset */
	reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_CPR);
	reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPR);

	reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPE);

	/* Enable Rx control. */
	val = reg_read(unicam, UNICAM_CTRL);
	set_field(&val, UNICAM_CPM_CSI2, UNICAM_CPM_MASK);
	set_field(&val, UNICAM_DCM_STROBE, UNICAM_DCM_MASK);

	/* Packet framer timeout */
	set_field(&val, 0xf, UNICAM_PFT_MASK);
	set_field(&val, 128, UNICAM_OET_MASK);
	reg_write(unicam, UNICAM_CTRL, val);

	reg_write(unicam, UNICAM_IHWIN, 0);
	reg_write(unicam, UNICAM_IVWIN, 0);

	/* AXI bus access QoS setup */
	val = reg_read(unicam, UNICAM_PRI);
	set_field(&val, 0, UNICAM_BL_MASK);
	set_field(&val, 0, UNICAM_BS_MASK);
	set_field(&val, 0xe, UNICAM_PP_MASK);
	set_field(&val, 8, UNICAM_NP_MASK);
	set_field(&val, 2, UNICAM_PT_MASK);
	set_field(&val, 1, UNICAM_PE);
	reg_write(unicam, UNICAM_PRI, val);

	reg_write_field(unicam, UNICAM_ANA, 0, UNICAM_DDL);

	val = UNICAM_FSIE | UNICAM_FEIE | UNICAM_IBOB;
	set_field(&val, line_int_freq, UNICAM_LCIE_MASK);
	reg_write(unicam, UNICAM_ICTL, val);
	reg_write(unicam, UNICAM_STA, UNICAM_STA_MASK_ALL);
	reg_write(unicam, UNICAM_ISTA, UNICAM_ISTA_MASK_ALL);

	/* tclk_term_en */
	reg_write_field(unicam, UNICAM_CLT, 2, UNICAM_CLT1_MASK);
	/* tclk_settle */
	reg_write_field(unicam, UNICAM_CLT, 6, UNICAM_CLT2_MASK);
	/* td_term_en */
	reg_write_field(unicam, UNICAM_DLT, 2, UNICAM_DLT1_MASK);
	/* ths_settle */
	reg_write_field(unicam, UNICAM_DLT, 6, UNICAM_DLT2_MASK);
	/* trx_enable */
	reg_write_field(unicam, UNICAM_DLT, 0, UNICAM_DLT3_MASK);

	reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_SOE);

	/* Packet compare setup - required to avoid missing frame ends */
	val = 0;
	set_field(&val, 1, UNICAM_PCE);
	set_field(&val, 1, UNICAM_GI);
	set_field(&val, 1, UNICAM_CPH);
	set_field(&val, 0, UNICAM_PCVC_MASK);
	set_field(&val, 1, UNICAM_PCDT_MASK);
	reg_write(unicam, UNICAM_CMP0, val);

	/* Enable clock lane and set up terminations */
	val = 0;
	/* CSI2 */

	set_field(&val, 1, UNICAM_CLE);
	set_field(&val, 1, UNICAM_CLLPE);
	// !NONCONTINOUS_CLOCK
	set_field(&val, 1, UNICAM_CLTRE);
	set_field(&val, 1, UNICAM_CLHSE);
	reg_write(unicam, UNICAM_CLK, val);

	/*
         * Enable required data lanes with appropriate terminations.
         * The same value needs to be written to UNICAM_DATn registers for
         * the active lanes, and 0 for inactive ones.
         */
	val = 0;
	/* CSI2 */
	set_field(&val, 1, UNICAM_DLE);
	set_field(&val, 1, UNICAM_DLLPE);
	set_field(&val, 1, UNICAM_DLTRE);
	set_field(&val, 1, UNICAM_DLHSE);

	reg_write(unicam, UNICAM_DAT0, val);

	if (unicam->active_data_lanes == 1)
		val = 0;
	reg_write(unicam, UNICAM_DAT1, val);

	// if (dev->max_data_lanes > 2) {
	/*
         * Registers UNICAM_DAT2 and UNICAM_DAT3 only valid if the
         * instance supports more than 2 data lanes.
         */
	if (unicam->active_data_lanes == 2)
		val = 0;
	reg_write(unicam, UNICAM_DAT2, val);

	if (unicam->active_data_lanes == 3)
		val = 0;
	reg_write(unicam, UNICAM_DAT3, val);
	// }

	// reg_write(dev, UNICAM_IBLS,
	//    dev->node[IMAGE_PAD].v_fmt.fmt.pix.bytesperline);
	// size = dev->node[IMAGE_PAD].v_fmt.fmt.pix.sizeimage;
	// reg_write(dev, UNICAM_IBLS, ALIGN(8, BPL_ALIGNMENT));
	reg_write(unicam, UNICAM_IBLS, unicam->hres);
	// unicam_wr_dma_addr(unicam, unicam->dummy_dma_addr, 0);
	unicam_schedule_next_block(unicam);
	unicam_set_packing_config(unicam);
	// const unsigned int raw8 = 0x2a;
	const unsigned int embedded_8bit_non_image_data = 0x12;
	unicam_cfg_image_id(unicam, embedded_8bit_non_image_data);

	val = reg_read(unicam, UNICAM_MISC);
	set_field(&val, 1, UNICAM_FL0);
	set_field(&val, 1, UNICAM_FL1);
	reg_write(unicam, UNICAM_MISC, val);

	/* Enable peripheral */
	reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_CPE);

	/* Load image pointers */
	reg_write_field(unicam, UNICAM_ICTL, 1, UNICAM_LIP_MASK);

	/*
         * Enable trigger only for the first frame to
         * sync correctly to the FS from the source.
         */
	reg_write_field(unicam, UNICAM_ICTL, 1, UNICAM_TFC);
}

static void unicam_disable(struct unicam_device *unicam)
{
	/* Analogue lane control disable */
	reg_write_field(unicam, UNICAM_ANA, 1, UNICAM_DDL);

	/* Stop the output engine */
	reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_SOE);

	/* Disable the data lanes. */
	reg_write(unicam, UNICAM_DAT0, 0);
	reg_write(unicam, UNICAM_DAT1, 0);

	if (unicam->active_data_lanes > 2) {
		reg_write(unicam, UNICAM_DAT2, 0);
		reg_write(unicam, UNICAM_DAT3, 0);
	}

	/* Peripheral reset */
	reg_write_field(unicam, UNICAM_CTRL, 1, UNICAM_CPR);
	usleep_range(50, 100);
	reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPR);

	/* Disable peripheral */
	reg_write_field(unicam, UNICAM_CTRL, 0, UNICAM_CPE);

	/* Clear ED setup */
	reg_write(unicam, UNICAM_DCS, 0);

	/* Disable all lane clocks */
	clk_write(unicam, 0);
}

static int unicam_log_status(struct unicam_device *unicam)
{
	struct device *dev;
	u32 reg;

	dev = &unicam->pdev->dev;
	reg = reg_read(unicam, UNICAM_IPIPE);

	dev_info(dev, "----Status log----\n");
	dev_info(dev, "Unpacking/packing:   %u / %u\n",
		 get_field(reg, UNICAM_PUM_MASK),
		 get_field(reg, UNICAM_PPM_MASK));
	dev_info(dev, "----Live data----\n");
	dev_info(dev, "Programmed stride:   %u\n",
		 reg_read(unicam, UNICAM_IBLS));
	dev_info(dev, "Detected resolution: %ux%u\n",
		 reg_read(unicam, UNICAM_IHSTA),
		 reg_read(unicam, UNICAM_IVSTA));
	dev_info(dev, "Write pointer:       %08x\n",
		 reg_read(unicam, UNICAM_IBWP));
	dev_info(dev, "STA:                 %08x\n",
		 reg_read(unicam, UNICAM_STA));
	dev_info(dev, "ISTA:                %08x\n",
		 reg_read(unicam, UNICAM_ISTA));
	dev_info(dev, "CTRL:                %08x\n",
		 reg_read(unicam, UNICAM_CTRL));
	return 0;
}

static u8 primes_read_mipi_lanes(struct unicam_device *unicam)
{
	u8 res = 2;
	if (unicam->fpga_i2c_mipi_config) {
		s32 data = i2c_smbus_read_byte_data(
				   unicam->fpga_i2c_mipi_config,
				   PRIMES_FPGA_I2C_ADDR_MIPI_LANES) +
			   1;
		if (data < 0) {
			dev_warn(
				&unicam->pdev->dev,
				"Could not read C_MIPI_TX_LANES from FPGA, using default of 2");
		} else {
			res = data;
		}
	}
	return res;
}

static u16 primes_read_mipi_hact(struct unicam_device *unicam)
{
	u16 res = 1024;
	if (unicam->fpga_i2c_mipi_config) {
		s32 data_0 = i2c_smbus_read_byte_data(
			unicam->fpga_i2c_mipi_config,
			PRIMES_FPGA_I2C_ADDR_MIPI_HACT);
		s32 data_1 = i2c_smbus_read_byte_data(
			unicam->fpga_i2c_mipi_config,
			PRIMES_FPGA_I2C_ADDR_MIPI_HACT + 1);
		if (data_0 < 0 || data_1 < 0) {
			dev_warn(
				&unicam->pdev->dev,
				"Could not read HACT from FPGA, using default of 1024");
		}
		res = data_0 << 8 | data_1;
	}
	return res;
}

static u16 primes_read_mipi_vact(struct unicam_device *unicam)
{
	u16 res = 4096;
	if (unicam->fpga_i2c_mipi_config) {
		s32 data_0 = i2c_smbus_read_byte_data(
			unicam->fpga_i2c_mipi_config,
			PRIMES_FPGA_I2C_ADDR_MIPI_VACT);
		s32 data_1 = i2c_smbus_read_byte_data(
			unicam->fpga_i2c_mipi_config,
			PRIMES_FPGA_I2C_ADDR_MIPI_VACT + 1);
		if (data_0 < 0 || data_1 < 0) {
			dev_warn(
				&unicam->pdev->dev,
				"Could not read C_MIPI_VACT from FPGA, using default of 4096");
		}
		res = data_0 << 8 | data_1;
	}
	return res;
}

static struct i2c_client *create_i2c_client_from_node(struct device *dev,
						      struct device_node *np)
{
	struct i2c_board_info board_info = {};
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	u32 addr;
	u32 busnum;

	if (of_property_read_u32(np, "reg", &addr)) {
		dev_err(dev, "missing 'reg' property in I2C node\n");
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32(np, "busnum", &busnum)) {
		dev_err(dev, "missing 'busnum' property in I2C node\n");
		return ERR_PTR(-EINVAL);
	}

	adapter = i2c_get_adapter(busnum);
	if (!adapter) {
		return ERR_PTR(-ENODEV);
	}

	strscpy(board_info.type, "primes-sensor", I2C_NAME_SIZE);
	board_info.addr = addr;
	board_info.of_node = np;

	client = i2c_new_client_device(adapter, &board_info);
	i2c_put_adapter(adapter);

	return client;
}

static int primes_connect_i2c_clients(struct unicam_device *unicam)
{
	int ret = 0;
	struct {
		const char *node_name;
		struct i2c_client **client;
	} nodes[] = {
		{ "fpga-i2c-mipi-config", &unicam->fpga_i2c_mipi_config },
		{ "fpga-i2c-amplifier-config",
		  &unicam->fpga_i2c_amplifier_config },
	};
	for (int i = 0; i < ARRAY_SIZE(nodes); i++) {
		struct device_node *sensor_np = of_get_child_by_name(
			unicam->pdev->dev.of_node, nodes[i].node_name);
		if (!sensor_np) {
			dev_err(&unicam->pdev->dev,
				"missing '%s' property in CSI node\n",
				nodes[i].node_name);
			return -EINVAL;
		}
		*nodes[i].client = create_i2c_client_from_node(
			&unicam->pdev->dev, sensor_np);
		if (IS_ERR(*nodes[i].client)) {
			dev_warn(&unicam->pdev->dev,
				 "failed to create i2c client from node\n");
			*nodes[i].client = NULL;
			ret = 1;
		}
		of_node_put(sensor_np);
		if (ret == 1) {
			break;
		}
	}
	if (ret == 1) {
		i2c_unregister_device(unicam->fpga_i2c_amplifier_config);
	}
	return ret;
}

static int unicam_start_streaming(struct unicam_device *unicam)
{
	int ret;
	mutex_lock(&unicam->mx_streaming);
	unicam->sequence = 0;
	ret = unicam_runtime_get(unicam);
	if (ret < 0) {
		dev_err(&unicam->pdev->dev, "unicam_runtime_get failed\n");
		goto err_streaming;
	}

	unicam->active_data_lanes = primes_read_mipi_lanes(unicam);
	unicam->hres = primes_read_mipi_hact(unicam);
	unicam->vres = primes_read_mipi_vact(unicam);

	dev_info(&unicam->pdev->dev, "Running with %u data lanes\n",
		 unicam->active_data_lanes);

	ret = pm_runtime_resume_and_get(&unicam->pdev->dev);
	if (ret < 0) {
		dev_err(&unicam->pdev->dev, "PM runtime resume failed: %d\n",
			ret);
		goto error_pipeline;
	}

	unicam->start_time = ktime_get_real_ns();

	unicam->frame_started = false;
	unicam_start_rx(unicam);
	unicam->streaming = true;

	ret = 0;
	goto end;

error_pipeline:
	pm_runtime_put_sync(&unicam->pdev->dev);
err_streaming:
end:
	mutex_unlock(&unicam->mx_streaming);
	return ret;
}

static void unicam_return_buffers(struct unicam_device *unicam)
{
	spin_lock(&unicam->list_lock);
	struct iio_dma_buffer_block *block, *tmp;
	list_for_each_entry_safe(block, tmp, &unicam->block_list, head) {
		list_del(&block->head);
		block->bytes_used = 0;
		iio_dma_buffer_block_done(block);
	}
	if (unicam->cur_block) {
		block->bytes_used = 0;
		iio_dma_buffer_block_done(unicam->cur_block);
	}
	if (unicam->next_block && unicam->cur_block != unicam->next_block) {
		block->bytes_used = 0;
		iio_dma_buffer_block_done(unicam->next_block);
	}
	unicam->cur_block = NULL;
	unicam->next_block = NULL;
	spin_unlock(&unicam->list_lock);
}

static void unicam_stop_streaming(struct unicam_device *unicam)
{
	mutex_lock(&unicam->mx_streaming);
	if (unicam->streaming) {
		pm_runtime_put_sync(&unicam->pdev->dev);
		unicam_disable(unicam);
		unicam_return_buffers(unicam);
		unicam->streaming = false;
	}
	mutex_unlock(&unicam->mx_streaming);
}

static void unicam_iio_buffer_release(struct iio_buffer *buf)
{
	struct iio_dma_buffer_queue *queue =
		container_of(buf, struct iio_dma_buffer_queue, buffer);
	iio_dma_buffer_release(queue);
}

static const struct iio_buffer_access_funcs unicam_iio_buffer_access_ops = {
	.read = iio_dma_buffer_read,
	.write = iio_dma_buffer_write,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.data_available = iio_dma_buffer_usage,
	.space_available = iio_dma_buffer_usage,
	.release = unicam_iio_buffer_release,
	.request_update = iio_dma_buffer_request_update,
	.enable = iio_dma_buffer_enable,
	.disable = iio_dma_buffer_disable,
	.set_length = iio_dma_buffer_set_length,

	.enqueue_dmabuf = iio_dma_buffer_enqueue_dmabuf,
	.attach_dmabuf = iio_dma_buffer_attach_dmabuf,
	.detach_dmabuf = iio_dma_buffer_detach_dmabuf,

	.lock_queue = iio_dma_buffer_lock_queue,
	.unlock_queue = iio_dma_buffer_unlock_queue,

	.modes = INDIO_BUFFER_HARDWARE,
	.flags = INDIO_BUFFER_FLAG_FIXED_WATERMARK,
};

static int unicam_dma_buffer_op_submit(struct iio_dma_buffer_queue *queue,
				       struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev;
	struct unicam_device *unicam;

	indio_dev = dev_get_drvdata(queue->dev);
	if (!indio_dev)
		return -ENODEV;

	unicam = iio_priv(indio_dev);
	if (!unicam)
		return -ENODEV;

	spin_lock(&unicam->list_lock);
	list_add_tail(&block->head, &unicam->block_list);
	spin_unlock(&unicam->list_lock);

	return 0;
}

static void unicam_dma_buffer_op_abort(struct iio_dma_buffer_queue *queue)
{
	dev_info(queue->dev, "dma buffer op abort");
}

static const struct iio_dma_buffer_ops unicam_iio_dma_buffer_ops = {
	.submit = unicam_dma_buffer_op_submit,
	.abort = unicam_dma_buffer_op_abort,
};

static const struct iio_chan_spec unicam_iio_channels[] = {
  {
    .type = IIO_COUNT,
    .scan_index = 0,
    .scan_type = {
      .sign = 'u',
      .realbits = 8,
      .storagebits = 8,
      .shift = 0,
      .repeat = 1,
      .endianness = IIO_LE,
    },
  }
};

struct primes_attribute {
	const char *name;
	u8 addr;
	bool two_bytes;
};

static int primes_i2c_write_u8(struct i2c_client *client, u8 addr, u16 val)
{
	return i2c_smbus_write_byte_data(client, addr, val & 0xff);
}

static int primes_i2c_read_u8(struct i2c_client *client, u8 addr)
{
	return i2c_smbus_read_byte_data(client, addr);
}

static int primes_i2c_write_u16(struct i2c_client *client, u8 addr, u16 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, addr, val >> 8);
	if (ret < 0) {
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, addr + 1, val);
	return ret;
}

static int primes_i2c_read_u16(struct i2c_client *client, u8 addr)
{
	int val, val2;
	val = i2c_smbus_read_byte_data(client, addr);
	if (val < 0) {
		return val;
	}
	val <<= 8;
	val2 = i2c_smbus_read_byte_data(client, addr + 1);
	if (val2 < 0) {
		return val2;
	}
	return (val & 0xff00) | (val2 & 0xff);
}

static inline struct unicam_device *dev_to_unicam(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	if (!indio_dev)
		return NULL;

	return iio_priv(indio_dev);
}

static ssize_t dac_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct iio_dev_attr *iioattr =
		container_of(attr, struct iio_dev_attr, dev_attr);
	struct unicam_device *unicam = dev_to_unicam(dev);
	u16 val = (u16)-1;

	if (!unicam)
		return -ENODEV;

	if (strcmp(attr->attr.name, "dac0:0") == 0) {
		val = primes_dac_read_data(&unicam->dac0, 0);
	} else if (strcmp(attr->attr.name, "dac0:1") == 0) {
		val = primes_dac_read_data(&unicam->dac0, 1);
	} else if (strcmp(attr->attr.name, "dac0:2") == 0) {
		val = primes_dac_read_data(&unicam->dac0, 2);
	} else if (strcmp(attr->attr.name, "dac0:3") == 0) {
		val = primes_dac_read_data(&unicam->dac0, 3);
	} else if (strcmp(attr->attr.name, "dac1:0") == 0) {
		val = primes_dac_read_data(&unicam->dac1, 0);
	} else if (strcmp(attr->attr.name, "dac1:1") == 0) {
		val = primes_dac_read_data(&unicam->dac1, 1);
	} else if (strcmp(attr->attr.name, "dac1:2") == 0) {
		val = primes_dac_read_data(&unicam->dac1, 2);
	} else if (strcmp(attr->attr.name, "dac1:3") == 0) {
		val = primes_dac_read_data(&unicam->dac1, 3);
	} else {
		return -EINVAL;
	}

	return sprintf(buf, "%u\n", val);
}

static ssize_t dac_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct iio_dev_attr *iioattr =
		container_of(attr, struct iio_dev_attr, dev_attr);
	struct unicam_device *unicam = dev_to_unicam(dev);
	u16 val;

	if (!unicam)
		return -ENODEV;

	if (kstrtou16(buf, 10, &val))
		return -EINVAL;

	val &= 0x0fff;

	if (strcmp(attr->attr.name, "dac0:0") == 0) {
		primes_dac_write_data(&unicam->dac0, 0, val);
	} else if (strcmp(attr->attr.name, "dac0:1") == 0) {
		primes_dac_write_data(&unicam->dac0, 1, val);
	} else if (strcmp(attr->attr.name, "dac0:2") == 0) {
		primes_dac_write_data(&unicam->dac0, 2, val);
	} else if (strcmp(attr->attr.name, "dac0:3") == 0) {
		primes_dac_write_data(&unicam->dac0, 3, val);
	} else if (strcmp(attr->attr.name, "dac1:0") == 0) {
		primes_dac_write_data(&unicam->dac1, 0, val);
	} else if (strcmp(attr->attr.name, "dac1:1") == 0) { /* fixed */
		primes_dac_write_data(&unicam->dac1, 1, val);
	} else if (strcmp(attr->attr.name, "dac1:2") == 0) {
		primes_dac_write_data(&unicam->dac1, 2, val);
	} else if (strcmp(attr->attr.name, "dac1:3") == 0) {
		primes_dac_write_data(&unicam->dac1, 3, val);
	} else {
		return -EINVAL;
	}

	return len;
}

static ssize_t dac0_common_config_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct iio_dev_attr *iioattr =
		container_of(attr, struct iio_dev_attr, dev_attr);
	struct primes_attribute *pattr =
		(struct primes_attribute *)iioattr->address;
	struct unicam_device *unicam = dev_to_unicam(dev);
	int ret;

	if (!unicam)
		return -ENODEV;

	ret = primes_dac_read_common_config(&unicam->dac0);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t primes_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct iio_dev_attr *iioattr =
		container_of(attr, struct iio_dev_attr, dev_attr);
	struct primes_attribute *pattr =
		(struct primes_attribute *)iioattr->address;
	struct unicam_device *unicam = dev_to_unicam(dev);
	struct i2c_client *client;

	if (!unicam)
		return -ENODEV;

	client = unicam->fpga_i2c_mipi_config;
	if (!client)
		return -ENXIO;

	if (pattr->two_bytes) {
		ret = primes_i2c_read_u16(client, pattr->addr);
	} else {
		ret = primes_i2c_read_u8(client, pattr->addr);
	}

	if (ret < 0) {
		dev_err(dev, "I2C read failed: %d\n", ret);
		return ret;
	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t primes_attr_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t len)
{
	int ret;
	struct iio_dev_attr *iioattr =
		container_of(attr, struct iio_dev_attr, dev_attr);
	struct primes_attribute *pattr =
		(struct primes_attribute *)iioattr->address;
	struct unicam_device *unicam = dev_to_unicam(dev);
	struct i2c_client *client;
	u16 val;

	if (!unicam)
		return -ENODEV;

	client = unicam->fpga_i2c_mipi_config;
	if (!client)
		return -ENXIO;

	if (kstrtou16(buf, 10, &val))
		return -EINVAL;

	if (pattr->two_bytes)
		ret = primes_i2c_write_u16(client, pattr->addr, val);
	else
		ret = primes_i2c_write_u8(client, pattr->addr, (u8)val);

	if (ret < 0) {
		dev_err(dev, "I2C write failed: %d\n", ret);
		return ret;
	}

	return len;
}

static ssize_t frames_lost_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct unicam_device *unicam = dev_to_unicam(dev);

	if (!unicam)
		return -ENODEV;

	return sprintf(buf, "%lld\n", unicam->frames_lost);
}

static ssize_t amp_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct unicam_device *unicam = dev_to_unicam(dev);
	u8 s0, s1;

	if (!unicam || !unicam->fpga_i2c_amplifier_config)
		return -ENXIO;

	s0 = primes_i2c_read_u8(unicam->fpga_i2c_amplifier_config, 0x14);
	s1 = primes_i2c_read_u8(unicam->fpga_i2c_amplifier_config, 0x15);

	if (!s1 && !s0)
		return sprintf(buf, "11x\n");
	else if (!s1 && s0)
		return sprintf(buf, "2x\n");

	return sprintf(buf, "unknown\n");
}

static ssize_t amp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct unicam_device *unicam = dev_to_unicam(dev);

	if (!unicam || !unicam->fpga_i2c_amplifier_config)
		return -ENXIO;

	if (sysfs_streq(buf, "11x")) {
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x14, 0);
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x15, 0);
	} else if (sysfs_streq(buf, "2x")) {
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x14, 1);
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x15, 0);
	} else {
		dev_warn(dev, "invalid amp value '%s'\n", buf);
		return -EINVAL;
	}

	return len;
}

static ssize_t amp_available_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "11x 2x\n");
}

static ssize_t tia_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct unicam_device *unicam = dev_to_unicam(dev);
	u8 s0, s1;

	if (!unicam || !unicam->fpga_i2c_amplifier_config)
		return -ENXIO;

	s0 = primes_i2c_read_u8(unicam->fpga_i2c_amplifier_config, 0x16);
	s1 = primes_i2c_read_u8(unicam->fpga_i2c_amplifier_config, 0x17);

	if (!s1 && !s0)
		return sprintf(buf, "max\n");
	else if (!s1 && s0)
		return sprintf(buf, "mid\n");
	else if (s1 && !s0)
		return sprintf(buf, "min\n");

	return sprintf(buf, "unknown\n");
}

static ssize_t tia_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct unicam_device *unicam = dev_to_unicam(dev);

	if (!unicam || !unicam->fpga_i2c_amplifier_config)
		return -ENXIO;

	if (sysfs_streq(buf, "max")) {
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x16, 0);
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x17, 0);
	} else if (sysfs_streq(buf, "mid")) {
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x16, 1);
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x17, 0);
	} else if (sysfs_streq(buf, "min")) {
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x16, 0);
		primes_i2c_write_u8(unicam->fpga_i2c_amplifier_config, 0x17, 1);
	} else {
		dev_warn(dev, "invalid tia value '%s'\n", buf);
		return -EINVAL;
	}

	return len;
}

static ssize_t tia_available_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "max mid min\n");
}

#define PRIMES_I2C_DEVICE_ATTR(NAME, ADDR, TWO_BYTES)                 \
	static IIO_DEVICE_ATTR(NAME, 0644, primes_attr_show,          \
			       primes_attr_store,                     \
			       ((intptr_t)&(struct primes_attribute){ \
				       .name = #NAME,                 \
				       .addr = ADDR,                  \
				       .two_bytes = TWO_BYTES,        \
			       }));

PRIMES_I2C_DEVICE_ATTR(test_pattern_config, 0x81, 0)
PRIMES_I2C_DEVICE_ATTR(vc, 0xa8, 0)
PRIMES_I2C_DEVICE_ATTR(type, 0xa9, 0)
PRIMES_I2C_DEVICE_ATTR(lanes, PRIMES_FPGA_I2C_ADDR_MIPI_LANES, 0)
PRIMES_I2C_DEVICE_ATTR(frame_mode, 0xab, 0)
PRIMES_I2C_DEVICE_ATTR(hact, PRIMES_FPGA_I2C_ADDR_MIPI_HACT, 1)
PRIMES_I2C_DEVICE_ATTR(vact, PRIMES_FPGA_I2C_ADDR_MIPI_VACT, 1)
PRIMES_I2C_DEVICE_ATTR(hsp, 0xb2, 1)
PRIMES_I2C_DEVICE_ATTR(hbp, 0xb4, 1)
PRIMES_I2C_DEVICE_ATTR(hfp, 0xb8, 1)
PRIMES_I2C_DEVICE_ATTR(vsp, 0xba, 1)
PRIMES_I2C_DEVICE_ATTR(vbp, 0xbc, 1)
PRIMES_I2C_DEVICE_ATTR(vfp, 0xc0, 1)

static IIO_DEVICE_ATTR_RO(frames_lost, 0);
static IIO_DEVICE_ATTR_RW(amp, 0);
static IIO_DEVICE_ATTR_RO(amp_available, 0);
static IIO_DEVICE_ATTR_RW(tia, 0);
static IIO_DEVICE_ATTR_RO(tia_available, 0);

// clang-format off
struct iio_dev_attr iio_dev_attr_dac0[] = {
	IIO_ATTR(dac0:0, 0644, dac_show, dac_store, 0),
	IIO_ATTR(dac0:1, 0644, dac_show, dac_store, 0),
	IIO_ATTR(dac0:2, 0644, dac_show, dac_store, 0),
	IIO_ATTR(dac0:3, 0644, dac_show, dac_store, 0)
};
struct iio_dev_attr iio_dev_attr_dac1[] = {
	IIO_ATTR(dac1:0, 0644, dac_show, dac_store, 0),
	IIO_ATTR(dac1:1, 0644, dac_show, dac_store, 0),
	IIO_ATTR(dac1:2, 0644, dac_show, dac_store, 0),
	IIO_ATTR(dac1:3, 0644, dac_show, dac_store, 0)
};
// clang-format on
static IIO_DEVICE_ATTR_RO(dac0_common_config, 0);

static struct attribute *my_attributes[] = {
	&iio_dev_attr_test_pattern_config.dev_attr.attr,
	&iio_dev_attr_vc.dev_attr.attr,
	&iio_dev_attr_type.dev_attr.attr,
	&iio_dev_attr_lanes.dev_attr.attr,
	&iio_dev_attr_frame_mode.dev_attr.attr,
	&iio_dev_attr_hsp.dev_attr.attr,
	&iio_dev_attr_hbp.dev_attr.attr,
	&iio_dev_attr_hact.dev_attr.attr,
	&iio_dev_attr_hfp.dev_attr.attr,
	&iio_dev_attr_vsp.dev_attr.attr,
	&iio_dev_attr_vbp.dev_attr.attr,
	&iio_dev_attr_vact.dev_attr.attr,
	&iio_dev_attr_vfp.dev_attr.attr,
	&iio_dev_attr_frames_lost.dev_attr.attr,
	&iio_dev_attr_amp.dev_attr.attr,
	&iio_dev_attr_amp_available.dev_attr.attr,
	&iio_dev_attr_tia.dev_attr.attr,
	&iio_dev_attr_tia_available.dev_attr.attr,
	&iio_dev_attr_dac0[0].dev_attr.attr,
	&iio_dev_attr_dac0[1].dev_attr.attr,
	&iio_dev_attr_dac0[2].dev_attr.attr,
	&iio_dev_attr_dac0[3].dev_attr.attr,
	&iio_dev_attr_dac1[0].dev_attr.attr,
	&iio_dev_attr_dac1[1].dev_attr.attr,
	&iio_dev_attr_dac1[2].dev_attr.attr,
	&iio_dev_attr_dac1[3].dev_attr.attr,
	&iio_dev_attr_dac0_common_config.dev_attr.attr,
	NULL
};

static const struct attribute_group my_attribute_group = {
	.attrs = my_attributes,
};

static const struct iio_info unicam_iio_info = {
	.attrs = &my_attribute_group,
};

static int unicam_buffer_postenable(struct iio_dev *indio_dev)
{
	dev_info(&indio_dev->dev, "unicam_buffer_postenable\n");
	struct unicam_device *unicam = iio_priv(indio_dev);
	int ret;

	if (!unicam)
		return -ENODEV;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	if (!unicam->fpga_i2c_mipi_config) {
		ret = -ENOSR;
		goto err_put_module;
	}

	unicam->cur_block = NULL;
	unicam->next_block = NULL;

	ret = unicam_start_streaming(unicam);
	if (ret)
		goto err_put_module;

	return 0;

err_put_module:
	module_put(THIS_MODULE);
	return ret;
}

static int unicam_buffer_predisable(struct iio_dev *indio_dev)
{
	dev_info(&indio_dev->dev, "unicam_buffer_predisable\n");
	struct unicam_device *unicam = iio_priv(indio_dev);

	if (!unicam)
		goto out_put;

	unicam_log_status(unicam);

	unicam_stop_streaming(unicam);

out_put:
	/* Must match try_module_get() in postenable */
	module_put(THIS_MODULE);
	return 0;
}

static int unicam_buffer_postdisable(struct iio_dev *indio_dev)
{
	dev_info(&indio_dev->dev, "unicam_buffer_postdisable\n");
	return 0;
}

struct iio_buffer_setup_ops unicam_buffer_setup_ops = {
	.postenable = unicam_buffer_postenable,
	.predisable = unicam_buffer_predisable,
	.postdisable = unicam_buffer_postdisable,
};

static int do_not_flash_fpga = 0;
module_param(do_not_flash_fpga, int, 0644);

static int unicam_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct unicam_device *unicam;
	int ret, irq;

	dev_info(dev, "unicam_probe\n");

	/* Allocate iio_dev with private data (unicam) attached */
	indio_dev = devm_iio_device_alloc(dev, sizeof(*unicam));
	if (!indio_dev)
		return -ENOMEM;

	unicam = iio_priv(indio_dev);
	memset(unicam, 0, sizeof(*unicam));

	mutex_init(&unicam->mx_streaming);
	unicam->frame_lost_reason = "unknown";

	unicam->pdev = pdev;
	unicam->indio_dev = indio_dev;

	/* Make indio_dev retrievable in remove() */
	platform_set_drvdata(pdev, indio_dev);

	/* DMA setup */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		dev_warn(dev, "Unable to set DMA mask (%d)\n", ret);

	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dma_parms = devm_kzalloc(dev, sizeof(*dev->dma_parms), GFP_KERNEL);
	if (!dev->dma_parms) {
		ret = -ENOMEM;
		goto err_clear_drvdata;
	}
	dma_set_max_seg_size(dev, UINT_MAX);

	/* Optional FPGA programming */
	if (!do_not_flash_fpga) {
		struct device_node *mgr_np;
		struct fpga_manager *mgr;
		struct fpga_image_info info = { 0 };

		mgr_np = of_parse_phandle(dev->of_node, "fpga-mgr", 0);
		if (!mgr_np) {
			dev_warn(dev, "no fpga-mgr property found\n");
			ret = -ENODEV;
			goto err_clear_drvdata;
		}

		mgr = of_fpga_mgr_get(mgr_np);
		of_node_put(mgr_np);
		if (IS_ERR(mgr)) {
			dev_err(dev, "failed to get FPGA manager: %pe\n", mgr);
			ret = -EPROBE_DEFER;
			goto err_clear_drvdata;
		}

		info.dev = dev;
		info.firmware_name = "efinix-t120.hex.bin";

		ret = fpga_mgr_load(mgr, &info);
		if (ret) {
			dev_err(dev, "failed to program FPGA (%d)\n", ret);
			fpga_mgr_put(mgr);
			goto err_clear_drvdata;
		}

		unicam->fpga_mgr = mgr;
	}

	/* I2C clients */
	ret = primes_connect_i2c_clients(unicam);
	if (ret) {
		ret = -EBUSY;
		goto err_put_fpga_mgr;
	}

	/* DAC setup */
	unicam->dac0.base = PRIMES_DAC0_BASE;
	unicam->dac0.client = unicam->fpga_i2c_amplifier_config;
	primes_dac_sdo_en(&unicam->dac0);
	primes_dac_write_common_config(&unicam->dac0, 585);

	unicam->dac1.base = PRIMES_DAC1_BASE;
	unicam->dac1.client = unicam->fpga_i2c_amplifier_config;
	primes_dac_sdo_en(&unicam->dac1);
	primes_dac_write_common_config(&unicam->dac1, 585);

	/* MMIO */
	unicam->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(unicam->base)) {
		dev_err(dev, "Failed to get main io block\n");
		ret = PTR_ERR(unicam->base);
		goto err_unreg_i2c;
	}

	unicam->clk_gate_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(unicam->clk_gate_base)) {
		dev_err(dev, "Failed to get 2nd io block\n");
		ret = PTR_ERR(unicam->clk_gate_base);
		goto err_unreg_i2c;
	}

	/* Clocks */
	unicam->clock = devm_clk_get(dev, "lp");
	if (IS_ERR(unicam->clock)) {
		dev_err(dev, "Failed to get lp clock\n");
		ret = PTR_ERR(unicam->clock);
		goto err_unreg_i2c;
	}

	unicam->vpu_clock = devm_clk_get(dev, "vpu");
	if (IS_ERR(unicam->vpu_clock)) {
		dev_err(dev, "Failed to get vpu clock\n");
		ret = PTR_ERR(unicam->vpu_clock);
		goto err_unreg_i2c;
	}

	/* IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "No IRQ resource\n");
		ret = irq;
		goto err_unreg_i2c;
	}

	ret = devm_request_irq(dev, irq, unicam_isr, 0, "unicam_capture0",
			       unicam);
	if (ret) {
		dev_err(dev, "Unable to request interrupt (%d)\n", ret);
		goto err_unreg_i2c;
	}

	/* Runtime PM */
	pm_runtime_enable(dev);

	/* Use devm-managed coherent allocation if possible */
	unicam->dummy_dma_vaddr = dmam_alloc_coherent(
		dev, DUMMY_BUF_SIZE, &unicam->dummy_dma_addr, GFP_KERNEL);
	unicam->dummy_dma_size = DUMMY_BUF_SIZE;
	if (!unicam->dummy_dma_vaddr) {
		dev_err(dev, "Unable to allocate dummy dma buffer\n");
		ret = -ENOMEM;
		goto err_pm_disable;
	}
	memset(unicam->dummy_dma_vaddr, 0xab, unicam->dummy_dma_size);

	INIT_LIST_HEAD(&unicam->block_list);
	spin_lock_init(&unicam->list_lock);

	/* IIO device setup */
	indio_dev->setup_ops = &unicam_buffer_setup_ops;
	indio_dev->name = "primes-iio-dev";
	indio_dev->info = &unicam_iio_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
	indio_dev->num_channels = ARRAY_SIZE(unicam_iio_channels);
	indio_dev->channels = unicam_iio_channels;

	for (int i = 0; i < ARRAY_SIZE(unicam->queue); i++) {
		iio_dma_buffer_init(&unicam->queue[i], dev,
				    &unicam_iio_dma_buffer_ops);
		unicam->queue[i].buffer.access = &unicam_iio_buffer_access_ops;
		unicam->queue[i].buffer.direction = IIO_BUFFER_DIRECTION_IN;

		iio_device_attach_buffer(indio_dev, &unicam->queue[i].buffer);
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Failed to register iio device: %d\n", ret);
		goto err_pm_disable;
	}

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);

err_unreg_i2c:
	if (unicam->fpga_i2c_mipi_config)
		i2c_unregister_device(unicam->fpga_i2c_mipi_config);
	if (unicam->fpga_i2c_amplifier_config)
		i2c_unregister_device(unicam->fpga_i2c_amplifier_config);

err_put_fpga_mgr:
	if (unicam->fpga_mgr) {
		if (!do_not_flash_fpga && unicam->fpga_mgr->mops &&
		    unicam->fpga_mgr->mops->fpga_remove)
			unicam->fpga_mgr->mops->fpga_remove(unicam->fpga_mgr);

		fpga_mgr_put(unicam->fpga_mgr);
		unicam->fpga_mgr = NULL;
	}

err_clear_drvdata:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int unicam_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct unicam_device *unicam;
	int ret;
	if (!indio_dev)
		return -ENODEV;
	unicam = iio_priv(indio_dev);
	/* 1) VPU clock: min-rate is optional on many platforms */
	ret = clk_set_min_rate(unicam->vpu_clock, MIN_VPU_CLOCK_RATE);
	if (ret) {
		if (ret == -EOPNOTSUPP || ret == -EINVAL) {
			dev_warn(
				dev,
				"VPU clock min_rate not supported (%d), continuing\n",
				ret);
		} else {
			dev_err(dev,
				"failed to set up VPU clock min_rate: %d\n",
				ret);
			return ret;
		}
	}
	ret = clk_prepare_enable(unicam->vpu_clock);
	if (ret) {
		dev_err(dev, "Failed to enable VPU clock: %d\n", ret);
		goto err_minrate;
	}
	/* 2) CSI clock: set rate (optional) then enable */
	if (clk_get_rate(unicam->clock) != 100 * 1000 * 1000) {
		ret = clk_set_rate(unicam->clock, 100 * 1000 * 1000);
		if (ret) {
			dev_err(dev, "failed to set up CSI clock rate: %d\n",
				ret);
			goto err_vpu;
		}
	}
	ret = clk_prepare_enable(unicam->clock);
	if (ret) {
		dev_err(dev, "failed to enable CSI clock: %d\n", ret);
		goto err_vpu;
	}
	return 0;

err_vpu:
	clk_disable_unprepare(unicam->vpu_clock);

err_minrate:
	/* Reset min_rate only if it was actually applied (best-effort) */
	ret = clk_set_min_rate(unicam->vpu_clock, 0);
	if (ret && ret != -EOPNOTSUPP && ret != -EINVAL)
		dev_warn(dev, "Failed to reset VPU clock min_rate: %d\n", ret);

	/* Return the original failure if enable failed; for min_rate unsupported we continued */
	return -EIO; /* replaced below in note */
}

static int unicam_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct unicam_device *unicam;

	if (!indio_dev)
		return 0;

	unicam = iio_priv(indio_dev);

	clk_disable_unprepare(unicam->clock);

	/* min_rate reset is best-effort */
	if (clk_set_min_rate(unicam->vpu_clock, 0) &&
	    clk_set_min_rate(unicam->vpu_clock, 0) != -EOPNOTSUPP &&
	    clk_set_min_rate(unicam->vpu_clock, 0) != -EINVAL) {
		dev_warn(dev, "Failed to reset the VPU clock min_rate\n");
	}

	clk_disable_unprepare(unicam->vpu_clock);

	return 0;
}

static void unicam_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct unicam_device *unicam;
	int i;

	dev_info(dev, "unicam_remove\n");

	indio_dev = platform_get_drvdata(pdev);
	if (!indio_dev)
		return;

	unicam = iio_priv(indio_dev);

	unicam_stop_streaming(unicam);

	pm_runtime_disable(dev);

	unicam_disable(unicam);
	usleep_range(1000, 2000);

	for (i = 0; i < ARRAY_SIZE(unicam->queue); i++)
		iio_dma_buffer_exit(&unicam->queue[i]);

	for (i = 0; i < ARRAY_SIZE(unicam->queue); i++)
		iio_dma_buffer_release(&unicam->queue[i]);

	/* Do NOT dma_free_coherent() here if you used dmam_alloc_coherent() */

	if (unicam->fpga_i2c_mipi_config) {
		i2c_unregister_device(unicam->fpga_i2c_mipi_config);
		unicam->fpga_i2c_mipi_config = NULL;
	}

	if (unicam->fpga_i2c_amplifier_config) {
		i2c_unregister_device(unicam->fpga_i2c_amplifier_config);
		unicam->fpga_i2c_amplifier_config = NULL;
	}

	if (unicam->fpga_mgr) {
		if (!do_not_flash_fpga && unicam->fpga_mgr->mops &&
		    unicam->fpga_mgr->mops->fpga_remove)
			unicam->fpga_mgr->mops->fpga_remove(unicam->fpga_mgr);

		fpga_mgr_put(unicam->fpga_mgr);
		unicam->fpga_mgr = NULL;
	}

	platform_set_drvdata(pdev, NULL);
}

static const struct dev_pm_ops unicam_pm_ops = { RUNTIME_PM_OPS(
	unicam_runtime_suspend, unicam_runtime_resume, NULL) };

static const struct of_device_id unicam_of_match[] = {
	{
		.compatible = "primes,bcm2835-unicam-primes-generic-fpga",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, unicam_of_match);

static struct platform_driver unicam_driver = {
  .probe    = unicam_probe,
  .remove   = unicam_remove,
  .driver = {
    .name = UNICAM_MODULE_NAME,
    .pm = pm_ptr(&unicam_pm_ops),
    .of_match_table = of_match_ptr(unicam_of_match),
  },
};

module_platform_driver(unicam_driver);

MODULE_AUTHOR("Julian Weiß <j.weiss@primes.de>");
MODULE_DESCRIPTION("PRIMES GmbH BCM2835 Unicam for generic FPGA sensor data");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(UNICAM_VERSION);

MODULE_IMPORT_NS(IIO_DMA_BUFFER);

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

// #include "bcm2835-iio-dma-buffer.h"
#include "vc4-regs-unicam.h"

#define UNICAM_MODULE_NAME "unicam"
#define UNICAM_VERSION "0.1.0"

#define BPL_ALIGNMENT 32
#define BYTES_PER_LINE ALIGN(1024, BPL_ALIGNMENT)
#define LINES_PER_FRAME 4096
#define BYTES_PER_IMAGE (LINES_PER_FRAME * BYTES_PER_LINE)
#define STRIDE (BYTES_PER_LINE)
#define DUMMY_BUF_SIZE PAGE_SIZE

/*
 * Unicam must request a minimum of 250Mhz from the VPU clock.
 * Otherwise the input FIFOs overrun and cause image corruption.
 */
#define MIN_VPU_CLOCK_RATE (250 * 1000 * 1000)

/*
 * Size of the dummy buffer allocation.
 *
 * Due to a HW bug causing buffer overruns in circular buffer mode under certain
 * (not yet fully known) conditions, the dummy buffer allocation is set to a
 * a single page size, but the hardware gets programmed with a buffer size of 0.
 */

enum pad_types { IMAGE_PAD, METADATA_PAD, MAX_NODES };

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
	unsigned int max_data_lanes;
	unsigned int active_data_lanes;

	bool frame_started;

	size_t dummy_dma_size;
	dma_addr_t dummy_dma_addr;
	void *dummy_dma_vaddr;

	struct iio_dev *indio_dev;
	struct iio_dma_buffer_queue queue[1];
	struct iio_dma_buffer_block *cur_block, *next_block;
	spinlock_t list_lock;
	struct list_head block_list;

	u64 total_bytes;
	u64 start_time;
	u64 total_frames;
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
			       unsigned int buffer_size, int pad_id)
{
	dma_addr_t endaddr = dmaaddr + buffer_size;
	if (pad_id == IMAGE_PAD) {
		reg_write(dev, UNICAM_IBSA0, dmaaddr);
		reg_write(dev, UNICAM_IBEA0, endaddr);
	} else {
		reg_write(dev, UNICAM_DBSA0, dmaaddr);
		reg_write(dev, UNICAM_DBEA0, endaddr);
	}
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
	unicam_wr_dma_addr(unicam, phys_addr, size, IMAGE_PAD);
	unicam->next_block = block;
	// printk("scheduling next_block: phys_addr=%llu size=%zu\n", phys_addr,
	//        size);
}

static void unicam_schedule_dummy_block(struct unicam_device *unicam)
{
	unicam_wr_dma_addr(unicam, unicam->dummy_dma_addr, 0, IMAGE_PAD);
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

static u64 seq = 0;

static irqreturn_t unicam_isr(int irq, void *dev)
{
	struct unicam_device *unicam = dev;
	u32 ista, sta;
	u32 ibwp, ibsa0, ibea0;
	bool fe;

	ibwp = reg_read(unicam, UNICAM_IBWP);
	sta = reg_read(unicam, UNICAM_STA);
	/* Write value back to clear the interrupts */
	reg_write(unicam, UNICAM_STA, sta);
	ista = reg_read(unicam, UNICAM_ISTA);
	/* Write value back to clear the interrupts */
	reg_write(unicam, UNICAM_ISTA, ista);
	ibsa0 = reg_read(dev, UNICAM_IBSA0);
	ibea0 = reg_read(dev, UNICAM_IBEA0);

	if (!(sta & (UNICAM_IS | UNICAM_PI0))) {
		return IRQ_HANDLED;
	}

	/*
	 * Look for either the Frame End interrupt or the Packet Capture status
	 * to signal a frame end.
	 */
	fe = (ista & UNICAM_FEI || sta & UNICAM_PI0);

	/*
         * We must run the frame end handler first. If we have a valid next_frm
         * and we get a simultaneout FE + FS interrupt, running the FS handler
         * first would null out the next_frm ptr and we would have lost the
         * buffer forever.
         */
	if (fe && unicam->frame_started) {
		unicam->frame_started = false;
		unicam->total_frames++;
		if (unicam->cur_block &&
		    unicam->cur_block != unicam->next_block) {
			unicam->cur_block->bytes_used = unicam->cur_block->size;
			unicam->total_bytes += BYTES_PER_IMAGE;
			unicam_process_block_done(unicam);
			unicam->cur_block = unicam->next_block;
			unicam->next_block = NULL;
		} else {
			unicam->cur_block = unicam->next_block;
		}
	}

	bool scheduled_dummy_buffer = false;
	if (ista & UNICAM_FSI) {
		unicam->frame_started = true;
		if (!unicam->next_block ||
		    unicam->cur_block == unicam->next_block) {
			unicam_schedule_dummy_block(unicam);
			scheduled_dummy_buffer = true;
		} else if (unicam->cur_block) {
			dev_warn(&unicam->pdev->dev, "lost some data\n");
			unicam->cur_block->bytes_used = 0;
			unicam_process_block_done(unicam);
			unicam->cur_block = unicam->next_block;
			unicam->next_block = NULL;
		}
	}

	char *reason = "unknown";

	// Schedule the next block
	if (ista & (UNICAM_FSI | UNICAM_LCI) && !fe) {
		spin_lock(&unicam->list_lock);
		if (!list_empty(&unicam->block_list) && !unicam->next_block) {
			scheduled_dummy_buffer = false;
			unicam_schedule_next_block(unicam);
		} else if (list_empty(&unicam->block_list)) {
			reason = "block_list is empty";
		} else if (unicam->next_block) {
			reason = "next_block is not empty";
		}
		spin_unlock(&unicam->list_lock);
	} else {
		reason = "FSI and FEI at the same time";
	}

	// if (scheduled_dummy_buffer) {
	// 	printk("============== %4llu ==============\n", seq);
	// 	printk("dummy buffer is left scheduled after %llu frames, reason: %s\n",
	// 	       seq - last_seq, reason);
	// 	if (ista & UNICAM_FEI) {
	// 		printk("ista & UNICAM_FEI\n");
	// 	}
	// 	if (sta & UNICAM_PI0) {
	// 		printk("sta & UNICAM_PI0\n");
	// 	}
	// 	if (ista & UNICAM_FSI) {
	// 		printk("ista & UNICAM_FSI\n");
	// 	}
	// 	if (ista & UNICAM_LCI) {
	// 		printk("ista & UNICAM_LCI\n");
	// 	}
	// 	last_seq = seq;
	// }

	seq++;
	// printk("==================================\n");

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

	int line_int_freq = unicam->queue[0].fileio.block_size / STRIDE;
	dev_info(
		&unicam->pdev->dev,
		"stride size: %d bytes, buffer size: %zu bytes -> setting line_int_freq to %d\n",
		STRIDE, unicam->queue[0].fileio.block_size, line_int_freq);

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
	// }
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
	reg_write(unicam, UNICAM_IBLS, STRIDE);
	unicam_wr_dma_addr(unicam, unicam->dummy_dma_addr, 0, IMAGE_PAD);
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

	if (unicam->max_data_lanes > 2) {
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

static int unicam_start_streaming(struct unicam_device *unicam)
{
	int ret;

	unicam->sequence = 0;
	ret = unicam_runtime_get(unicam);
	if (ret < 0) {
		dev_err(&unicam->pdev->dev, "unicam_runtime_get failed\n");
		goto err_streaming;
	}

	unicam->max_data_lanes = 2;
	unicam->active_data_lanes = unicam->max_data_lanes;

	dev_info(&unicam->pdev->dev, "Running with %u data lanes\n",
		 unicam->active_data_lanes);

	ret = pm_runtime_resume_and_get(&unicam->pdev->dev);
	if (ret < 0) {
		dev_err(&unicam->pdev->dev, "PM runtime resume failed: %d\n",
			ret);
		goto error_pipeline;
	}

	unicam->start_time = ktime_get_real_ns();

	unicam_start_rx(unicam);

	return 0;

error_pipeline:
	pm_runtime_put_sync(&unicam->pdev->dev);
err_streaming:

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
	pm_runtime_put_sync(&unicam->pdev->dev);
	unicam_disable(unicam);
	unicam_return_buffers(unicam);
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
	struct platform_device *pdev =
		container_of(queue->dev, struct platform_device, dev);
	struct unicam_device *unicam = platform_get_drvdata(pdev);
	spin_lock(&unicam->list_lock);
	list_add_tail(&block->head, &unicam->block_list);
	spin_unlock(&unicam->list_lock);
	return 0;
}

static void unicam_dma_buffer_op_abort(struct iio_dma_buffer_queue *queue)
{
}

static const struct iio_dma_buffer_ops unicam_iio_dma_buffer_ops = {
	.submit = unicam_dma_buffer_op_submit,
	.abort = unicam_dma_buffer_op_abort,
};

static const struct iio_chan_spec unicam_iio_channels[] = {
  {
    .type = IIO_VOLTAGE,
    .scan_index = 0,
    .scan_type = {
      .sign = 'u',
      .realbits = 16,
      .storagebits = 16,
      .shift = 0,
      .repeat = 2,
      .endianness = IIO_LE,
    },
  },
  {
    .type = IIO_COUNT,
    .scan_index = 1,
    .scan_type = {
      .sign = 'u',
      .realbits = 32,
      .storagebits = 32,
      .shift = 0,
      .repeat = 1,
      .endianness = IIO_LE,
    },
  },
};

static const struct iio_info unicam_iio_info = {};

static int unicam_buffer_postenable(struct iio_dev *indio_dev)
{
	int res;
	struct platform_device *pdev;
	struct unicam_device *unicam;
	pdev = container_of(indio_dev->dev.parent, struct platform_device, dev);
	unicam = platform_get_drvdata(pdev);
	unicam->cur_block = NULL;
	unicam->next_block = NULL;
	res = unicam_start_streaming(unicam);
	unicam_log_status(unicam);
	return res;
}

static int unicam_buffer_predisable(struct iio_dev *indio_dev)
{
	struct platform_device *pdev = container_of(
		indio_dev->dev.parent, struct platform_device, dev);
	struct unicam_device *unicam = platform_get_drvdata(pdev);
	unicam_log_status(unicam);
	printk("predisable\n");
	unicam_stop_streaming(unicam);
	return 0;
}

static int unicam_buffer_postdisable(struct iio_dev *indio_dev)
{
	return 0;
}

struct iio_buffer_setup_ops unicam_buffer_setup_ops = {
	.postenable = unicam_buffer_postenable,
	.predisable = unicam_buffer_predisable,
	.postdisable = unicam_buffer_postdisable,
};

static int unicam_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct unicam_device *unicam;
	int ret;

	dev = &pdev->dev;
	dev_info(dev, "unicam_probe\n");

	unicam = kzalloc(sizeof(*unicam), GFP_KERNEL);
	if (!unicam)
		return -ENOMEM;

	unicam->pdev = pdev;
	platform_set_drvdata(pdev, unicam);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		printk("Unable to set DMA mask\n");
	}
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_parms = devm_kzalloc(
		&pdev->dev, sizeof(*pdev->dev.dma_parms), GFP_KERNEL);
	if (!pdev->dev.dma_parms) {
		return -ENOMEM;
	}
	dma_set_max_seg_size(&pdev->dev, UINT_MAX);

	/*
         * Adopt the current setting of the module parameter, and check if
         * device tree requests it.
         */

	unicam->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(unicam->base)) {
		dev_err(dev, "Failed to get main io block\n");
		ret = PTR_ERR(unicam->base);
		goto err_unicam_put;
	}

	unicam->clk_gate_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(unicam->clk_gate_base)) {
		dev_err(dev, "Failed to get 2nd io block\n");
		ret = PTR_ERR(unicam->clk_gate_base);
		goto err_unicam_put;
	}

	unicam->clock = devm_clk_get(&pdev->dev, "lp");
	if (IS_ERR(unicam->clock)) {
		dev_err(dev, "Failed to get lp clock\n");
		ret = PTR_ERR(unicam->clock);
		goto err_unicam_put;
	}

	unicam->vpu_clock = devm_clk_get(&pdev->dev, "vpu");
	if (IS_ERR(unicam->vpu_clock)) {
		dev_err(dev, "Failed to get vpu clock\n");
		ret = PTR_ERR(unicam->vpu_clock);
		goto err_unicam_put;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(dev, "No IRQ resource\n");
		ret = -EINVAL;
		goto err_unicam_put;
	}

	ret = devm_request_irq(&pdev->dev, ret, unicam_isr, 0,
			       "unicam_capture0", unicam);
	if (ret) {
		dev_err(dev, "Unable to request interrupt\n");
		ret = -EINVAL;
		goto err_unicam_put;
	}

	/* Enable the block power domain */
	pm_runtime_enable(&pdev->dev);

	unicam->dummy_dma_vaddr = dma_alloc_coherent(&pdev->dev, DUMMY_BUF_SIZE,
						     &unicam->dummy_dma_addr,
						     GFP_KERNEL);
	unicam->dummy_dma_size = DUMMY_BUF_SIZE; // This data is not needed
	if (!unicam->dummy_dma_vaddr) {
		dev_err(dev, "Unable to allocated dma buffer\n");
		ret = -EINVAL;
		goto err_unicam_put;
	}

	INIT_LIST_HEAD(&unicam->block_list);
	spin_lock_init(&unicam->list_lock);

	unicam->indio_dev = devm_iio_device_alloc(&unicam->pdev->dev, 0);
	unicam->indio_dev->priv = unicam;
	unicam->indio_dev->setup_ops = &unicam_buffer_setup_ops;
	unicam->indio_dev->name = "primes-iio-dev";
	unicam->indio_dev->info = &unicam_iio_info;
	unicam->indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
	unicam->indio_dev->num_channels = ARRAY_SIZE(unicam_iio_channels);
	unicam->indio_dev->channels = unicam_iio_channels;

	for (int i = 0; i < ARRAY_SIZE(unicam->queue); i++) {
		iio_dma_buffer_init(&unicam->queue[i], &unicam->pdev->dev,
				    &unicam_iio_dma_buffer_ops);
		unicam->queue[i].buffer.access = &unicam_iio_buffer_access_ops;
		unicam->queue[i].buffer.direction = IIO_BUFFER_DIRECTION_IN;
		iio_device_attach_buffer(unicam->indio_dev,
					 &unicam->queue[i].buffer);
	}

	ret = iio_device_register(unicam->indio_dev);

	if (ret) {
		dev_err(dev, "Failed to register iio device: %u\n", ret);
		return 1;
	}

	return 0;

err_unicam_put:
	kfree(unicam);

	return ret;
}

static void unicam_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "unicam_remove\n");
	struct unicam_device *unicam = platform_get_drvdata(pdev);
	if (unicam) {
		unicam_disable(unicam);
		usleep_range(1000, 2000);
		if (unicam->indio_dev) {
			for (int i = 0; i < ARRAY_SIZE(unicam->queue); i++) {
				iio_dma_buffer_exit(&unicam->queue[i]);
			}
			for (int i = 0; i < ARRAY_SIZE(unicam->queue); i++) {
				iio_dma_buffer_release(&unicam->queue[i]);
			}
			iio_device_unregister(unicam->indio_dev);
		}
		if (unicam->dummy_dma_vaddr) {
			dma_free_coherent(&pdev->dev, unicam->dummy_dma_size,
					  unicam->dummy_dma_vaddr,
					  unicam->dummy_dma_addr);
		}
		kfree(unicam);
	}
}

static int unicam_runtime_resume(struct device *dev)
{
	struct unicam_device *unicam = dev_get_drvdata(dev);
	int ret;
	ret = clk_set_min_rate(unicam->vpu_clock, MIN_VPU_CLOCK_RATE);
	if (ret) {
		dev_err(dev, "failed to set up VPU clock\n");
		return ret;
	}
	ret = clk_prepare_enable(unicam->vpu_clock);
	if (ret) {
		dev_err(dev, "Failed to enable VPU clock: %d\n", ret);
		goto err_vpu_clock;
	}
	ret = clk_set_rate(unicam->clock, 100 * 1000 * 1000);
	if (ret) {
		dev_err(dev, "failed to set up CSI clock\n");
		goto err_vpu_prepare;
	}
	ret = clk_prepare_enable(unicam->clock);
	if (ret) {
		dev_err(dev, "failed to enable CSI clock: %d\n", ret);
		goto err_vpu_prepare;
	}
	return 0;

err_vpu_prepare:
	clk_disable_unprepare(unicam->vpu_clock);
err_vpu_clock:
	if (clk_set_min_rate(unicam->vpu_clock, 0)) {
		dev_err(dev, "Failed to reset the VPU clock\n");
	}

	return ret;
}

static int unicam_runtime_suspend(struct device *dev)
{
	struct unicam_device *unicam = dev_get_drvdata(dev);
	clk_disable_unprepare(unicam->clock);
	if (clk_set_min_rate(unicam->vpu_clock, 0)) {
		dev_warn(dev, "Failed to reset the VPU clock\n");
	}
	clk_disable_unprepare(unicam->vpu_clock);
	return 0;
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

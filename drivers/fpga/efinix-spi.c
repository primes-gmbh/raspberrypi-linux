// SPDX-License-Identifier: GPL-2.0-only
/*
 * Efinix passive SPI driver
 *
 * Copyright (C) 2025 PRIMES GmbH
 *
 * Julian Weiß <j.weiss@primes.de>
 *
 * Manage Efinix FPGA firmware that is loaded over SPI using
 * the passive serial configuration interface.
 */

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>

struct efinix_fpga_mgr {
	struct spi_device *spi;
	struct gpio_desc *cdone;
	struct gpio_desc *creset;
	struct fpga_manager *mgr;
};

static void efinix_fpga_remove(struct fpga_manager *mgr)
{
	struct efinix_fpga_mgr *efx = mgr->priv;
	gpiod_set_value_cansleep(efx->creset, 0);
	msleep(1);
	gpiod_set_value_cansleep(efx->creset, 1);
}

static int efinix_fpga_write_init(struct fpga_manager *mgr,
				  struct fpga_image_info *info, const char *buf,
				  size_t count)
{
	int ret = 0;
	struct efinix_fpga_mgr *efx = mgr->priv;
	if (gpiod_get_value_cansleep(efx->cdone)) {
		dev_info(&efx->spi->dev,
			 "FPGA already programmed, reflashing...\n");
	} else {
		dev_info(&efx->spi->dev,
			 "FPGA not programmed yet, flashing...\n");
	}
	struct spi_message msg;
	struct spi_transfer assert_cs = { .cs_change = 1 };
	spi_bus_lock(efx->spi->controller);
	spi_message_init(&msg);
	spi_message_add_tail(&assert_cs, &msg);
	dev_info(&efx->spi->dev, "pulling CRESET to 0\n");
	gpiod_set_value_cansleep(efx->creset, 0);
	msleep(1);
	dev_info(&efx->spi->dev, "pulling CS to 0\n");
	ret = spi_sync_locked(efx->spi, &msg);
	if (ret) {
		dev_err(&efx->spi->dev, "error while driving CS\n");
		goto fail_unlock;
	}
	msleep(1);
	dev_info(&efx->spi->dev, "pulling CRESET to 1\n");
	gpiod_set_value_cansleep(efx->creset, 1);
	msleep(1);
	goto exit;

fail_unlock:
	spi_bus_unlock(efx->spi->controller);
exit:
	return ret;
}

static int efinix_fpga_write(struct fpga_manager *mgr, const char *buf,
			     size_t count)
{
	int ret = 0;
	struct efinix_fpga_mgr *efx = mgr->priv;
	dev_info(&efx->spi->dev, "Writing to FPGA...\n");
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = buf,
		.len = count,
		.cs_change = 1,
	};
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync_locked(efx->spi, &msg);
	if (ret) {
		dev_err(&efx->spi->dev, "SPI error in firmware write: %d\n",
			ret);
		goto fail_unlock;
	}
	goto exit;

fail_unlock:
	spi_bus_unlock(efx->spi->controller);

exit:
	return ret;
}

static enum fpga_mgr_states efinix_fpga_state(struct fpga_manager *mgr)
{
	struct efinix_fpga_mgr *efx = mgr->priv;
	enum fpga_mgr_states res = FPGA_MGR_STATE_OPERATING;
	if (efx->cdone) {
		if (!gpiod_get_value_cansleep(efx->cdone)) {
			res = FPGA_MGR_STATE_RESET;
		}
	}
	return res;
}

static int efinix_fpga_write_complete(struct fpga_manager *mgr,
				      struct fpga_image_info *info)
{
	int res;
	struct efinix_fpga_mgr *efx = mgr->priv;
	dev_info(&efx->spi->dev, "completed writing to FPGA\n");
	struct spi_message msg;
	char buf[32] = { 0 };
	struct spi_transfer clk_cycles = {
		.len = sizeof(buf),
		.tx_buf = buf,
	};
	spi_message_init(&msg);
	spi_message_add_tail(&clk_cycles, &msg);
	dev_info(&efx->spi->dev, "pulling CS to 1\n");
	res = spi_sync_locked(efx->spi, &msg);
	if (res) {
		dev_err(&efx->spi->dev, "SPI write failed\n");
	}
	spi_bus_unlock(efx->spi->controller);
	if (res) {
		return res;
	}
	res = ETIMEDOUT;
	if (efx->cdone) {
		int retries = 200;
		while (retries--) {
			if (gpiod_get_value_cansleep(efx->cdone)) {
				dev_info(&efx->spi->dev,
					 "FPGA configuration DONE\n");
				res = 0;
				break;
			}
			usleep_range(1000, 2000);
		}
		if (res) {
			dev_err(&efx->spi->dev, "FPGA CDONE did not go high\n");
		}
	} else {
		res = 0;
		usleep_range(1000, 2000);
	}
	return res;
}

static ssize_t reflash_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct efinix_fpga_mgr *efx = dev_get_drvdata(dev);
	struct fpga_image_info info = { 0 };
	info.dev = dev;
	info.firmware_name = (char *)buf;
	fpga_mgr_load(efx->mgr, &info);
	return count;
}

static DEVICE_ATTR_WO(reflash);

static struct attribute *efinix_attrs[] = {
	&dev_attr_reflash.attr,
	NULL,
};

ATTRIBUTE_GROUPS(efinix);

static const struct fpga_manager_ops efinix_fpga_ops = {
	.state = efinix_fpga_state,
	.write_init = efinix_fpga_write_init,
	.write = efinix_fpga_write,
	.write_complete = efinix_fpga_write_complete,
	.fpga_remove = efinix_fpga_remove,
	.groups = efinix_groups,
};

static int efinix_spi_probe(struct spi_device *spi)
{
	struct efinix_fpga_mgr *efx;
	struct fpga_manager_info info = { 0 };
	efx = devm_kzalloc(&spi->dev, sizeof(*efx), GFP_KERNEL);
	if (!efx) {
		return -ENOMEM;
	}
	efx->spi = spi;
	efx->cdone = devm_gpiod_get_optional(&spi->dev, "cdone", GPIOD_IN);
	if (IS_ERR(efx->cdone)) {
		return dev_err_probe(&efx->spi->dev, PTR_ERR(efx->cdone),
				     "failed to get CDONE gpio\n");
	}
	efx->creset = devm_gpiod_get(&spi->dev, "creset", GPIOD_OUT_HIGH);
	if (IS_ERR(efx->creset)) {
		return dev_err_probe(&efx->spi->dev, PTR_ERR(efx->creset),
				     "failed to get CRESET gpio\n");
	}
	info.name = "Efinix FPGA Manager";
	info.mops = &efinix_fpga_ops;
	info.priv = efx;
	efx->mgr = devm_fpga_mgr_register_full(&spi->dev, &info);
	if (IS_ERR(efx->mgr)) {
		return dev_err_probe(&efx->spi->dev, PTR_ERR(efx->mgr),
				     "failed to register FPGA manager");
	}
	dev_set_drvdata(&efx->mgr->dev, efx);
	return 0;
}

static const struct of_device_id efinix_spi_of_match[] = {
	{
		.compatible = "efinix,fpga-passive-spi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, efinix_spi_of_match);

static const struct spi_device_id efinix_fpga_ids[] = { { "fpga-passive-spi",
							  0 },
							{ /* sentinel */ } };
MODULE_DEVICE_TABLE(spi, efinix_fpga_ids);

static struct spi_driver efinix_spi_driver = {
	.driver = {
		.name = "efinix-spi",
		.of_match_table = of_match_ptr(efinix_spi_of_match),
	},
	.probe = efinix_spi_probe,
	.id_table = efinix_fpga_ids,
};

module_spi_driver(efinix_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Julian Weiß <j.weiss@primes.de>");
MODULE_DESCRIPTION("Efinix FPGA configuration via passive SPI");

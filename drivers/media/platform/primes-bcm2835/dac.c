#include "dac.h"

static void primes_dac_write(struct primes_dac *dac, u8 addr, u16 data)
{
	i2c_smbus_write_byte_data(dac->client, dac->base + PRIMES_DAC_OFF_TX,
				  data & 0xff);
	i2c_smbus_write_byte_data(dac->client,
				  dac->base + PRIMES_DAC_OFF_TX + 1,
				  (data >> 8) & 0xff);
	i2c_smbus_write_byte_data(dac->client,
				  dac->base + PRIMES_DAC_OFF_TX + 2, addr);
}

static u16 primes_dac_read(struct primes_dac *dac, u8 addr)
{
	int val, val2;
	i2c_smbus_write_byte_data(
		dac->client, dac->base + PRIMES_DAC_OFF_TX + 2, addr + 128);
	i2c_smbus_write_byte_data(dac->client,
				  dac->base + PRIMES_DAC_OFF_TX + 2, 0);
	val = i2c_smbus_read_byte_data(dac->client,
				       dac->base + PRIMES_DAC_OFF_RX);
	if (val < 0) {
		return val;
	}
	val2 = i2c_smbus_read_byte_data(dac->client,
					 dac->base + PRIMES_DAC_OFF_RX + 1);
	if (val2 < 0) {
		return val2;
	}
	return (val & 0xff) | ((val2 & 0xff) << 8);
}

void primes_dac_sdo_en(struct primes_dac *dac)
{
	primes_dac_write(dac, PRIMES_DAC_ADDR_SDO, 1);
}

u16 primes_dac_read_common_config(struct primes_dac *dac)
{
	return primes_dac_read(dac, PRIMES_DAC_ADDR_COMMON_CONFIG);
}

void primes_dac_write_common_config(struct primes_dac *dac, u16 data)
{
	primes_dac_write(dac, PRIMES_DAC_ADDR_COMMON_CONFIG, data);
}

void primes_dac_write_data(struct primes_dac *dac, u8 i, u16 data)
{
	if (i < 4) {
		primes_dac_write(dac, PRIMES_DAC_ADDR_DATA + i, data << 4);
	}
}

u16 primes_dac_read_data(struct primes_dac *dac, u8 i)
{
	if (i < 4) {
		return primes_dac_read(dac, PRIMES_DAC_ADDR_DATA + i) >> 4;
	}
	return -1;
}

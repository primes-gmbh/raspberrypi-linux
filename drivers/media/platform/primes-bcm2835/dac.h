#ifndef DAC_HPP
#define DAC_HPP

#include <asm-generic/types.h>
#include <linux/i2c.h>

#define PRIMES_DAC0_BASE 0x00
#define PRIMES_DAC1_BASE 0x08
#define PRIMES_DAC_OFF_TX 1
#define PRIMES_DAC_OFF_RX 4
#define PRIMES_DAC_ADDR_SDO 0x26
#define PRIMES_DAC_ADDR_COMMON_CONFIG 0x1f
#define PRIMES_DAC_ADDR_DATA 0x19

struct primes_dac {
	struct i2c_client *client;
	u8 base;
};

void primes_dac_sdo_en(struct primes_dac *dac);
u16 primes_dac_read_common_config(struct primes_dac *dac);
void primes_dac_write_common_config(struct primes_dac *dac, u16 data);
void primes_dac_write_data(struct primes_dac *dac, u8 i, u16 data);
u16 primes_dac_read_data(struct primes_dac *dac, u8 i);

#endif

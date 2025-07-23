#include <util/twi.h>

void i2c_init(void);

void i2c_start(void);

void i2c_stop(void);

uint8_t i2c_write(float data);

uint8_t i2c_read_ack(void);

uint8_t i2c_read_nack(void);
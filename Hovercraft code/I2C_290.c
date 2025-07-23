#include <util/twi.h>
volatile uint8_t I2C_status;
void i2c_init(void) {
    // Set SCL frequency to 400kHz (assuming F_CPU = 16MHz)
    TWSR = 0x00; // Prescaler value
    TWBR = 12;   // Bit rate register value for 400kHz
}

void i2c_start(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    // if (((TWSR&0xF8)!=TW_START)&&((TWSR&0xF8)!=TW_REP_START)) return 1; 

    // i2c_addr =((i2c_addr<<1)|(read_write&1));
    // TWDR=i2c_addr;
    // TWCR=((1<<TWINT)|(1<<TWEN));

    // while(!(TWCR&(1<<TWINT)));
    // if (((TWSR&0xF8)!=TW_MT_SLA_ACK)&&((TWSR&0xF8)!=TW_MR_SLA_ACK)) return 2;
    // return 0;
}

void i2c_stop(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    while(TWCR & (1<<TWSTO));
}

uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    if((TWSR&0xF8)!=TW_MT_DATA_ACK) return 1; 
    return 0;
}

uint8_t i2c_read_ack(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t i2c_read_nack(void) {
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}
#ifndef __DEMO_I2C_H
#define __DEMO_I2C_H

#include "stm8s.h"

#define I2C_SPEED 100000  // (hz)  standard mode speed

#define I2C_ADDRESS 0x50

#define MAX_BUFFER  32


void I2C_transaction_begin(void);

void I2C_transaction_end(void);

void I2C_byte_received(u8 u8_RxData);

u8 I2C_byte_write(void);


void I2C_Slave_Init(uint8_t address);

#endif
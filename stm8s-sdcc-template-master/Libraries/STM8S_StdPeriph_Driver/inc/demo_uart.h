#ifndef __DEMO_UART_H
#define __DEMO_UART_H

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_uart1.h"

void uart1_config(void);

int uart_write(const char *str);
void UART1_SendByte(uint8_t byte);
void UART1_SendString(const char *str);
void UART1_SendRegisterValue(const char *regName, uint8_t regValue);
void UART1_Send16(uint16_t value) ;

#endif
/********************************************************************
 * Copyright 2017 Ahmet Onat
 * This program is distributed under the terms of the 
 * GNU General Public License
 *
 * This file is part of SDCC_StmPeriphLib
 *
 * SDCC_StmPeriphLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SDCC_StmPeriphLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with SDCC_StmPeriphLib.  If not, see <http://www.gnu.org/licenses/>.
 *
 *********************************************************************/


#include "stm8s.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_clk.h"
#include "stm8s_exti.h"
#include "stm8s_uart1.h"
#include <string.h> 
#include <stdio.h>

/* I2C Setting */
#define I2C_SPEED 100000  // (hz)  standard mode speed
#define I2C_ADDRESS 0x50

/* GPIO Setting */
#define LED_PORT  GPIOB
#define LED_PIN   GPIO_PIN_5

#define DELAYTIME 0xF000
void delay (uint16_t count);

void I2C_Setup(void);
void I2C_ByteWrite(u8 I2C_Slave_Address, u8 iData);
void I2C_Slave_Init(uint8_t address);

// uart
int uart_write(const char *str);
void UART1_SendByte(uint8_t byte);
void UART1_SendString(const char *str);
void UART1_SendRegisterValue(const char *regName, uint8_t regValue);

// i2c
#define MAX_BUFFER  32

   u8 u8_My_Buffer[MAX_BUFFER];
   u8 *u8_MyBuffp;
   u8 MessageBegin;

	void I2C_transaction_begin(void)
	{
		MessageBegin = TRUE;
	}
	void I2C_transaction_end(void)
	{
		//Not used in this example
	}
	void I2C_byte_received(u8 u8_RxData)
	{
		if (MessageBegin == TRUE  &&  u8_RxData < MAX_BUFFER) {
			u8_MyBuffp= &u8_My_Buffer[u8_RxData];
			MessageBegin = FALSE;
		}
    else if(u8_MyBuffp < &u8_My_Buffer[MAX_BUFFER])
      *(u8_MyBuffp++) = u8_RxData;
	}
	u8 I2C_byte_write(void)
	{
		if (u8_MyBuffp < &u8_My_Buffer[MAX_BUFFER])
			return *(u8_MyBuffp++);
		else
			return 0x00;
	}



// i2c 
void I2C_IRQHandler(void) __interrupt(19) {
  GPIO_WriteReverse(GPIOD,GPIO_PIN_4);

	static u8 sr1;					
	static u8 sr2;
	static u8 sr3;
	
// save the I2C registers configuration
sr1 = I2C->SR1;
sr2 = I2C->SR2;
sr3 = I2C->SR3;

/* Communication error? */
  if (sr2 & (I2C_SR2_WUFH | I2C_SR2_OVR |I2C_SR2_ARLO |I2C_SR2_BERR))
  {		
    uart_write("Condition1 \r\n");
    I2C->CR2|= I2C_CR2_STOP;  // stop communication - release the lines
    I2C->SR2= 0;					    // clear all error flags
	}
/* More bytes received ? */
  if ((sr1 & (I2C_SR1_RXNE | I2C_SR1_BTF)) == (I2C_SR1_RXNE | I2C_SR1_BTF))
  {
    uart_write("Condition2 \r\n");
    UART1_SendRegisterValue("C2", I2C->DR);
    I2C_byte_received(I2C->DR);
  }
/* Byte received ? */
  if (sr1 & I2C_SR1_RXNE)
  {
    uart_write("Condition3 \r\n");
    UART1_SendRegisterValue("C3", I2C->DR);
    I2C_byte_received(I2C->DR);
  }
/* NAK? (=end of slave transmit comm) */
  if (sr2 & I2C_SR2_AF)
  {	
    uart_write("Condition4 \r\n");
    I2C->SR2 &= ~I2C_SR2_AF;	  // clear AF
		I2C_transaction_end();
	}
/* Stop bit from Master  (= end of slave receive comm) */
  if (sr1 & I2C_SR1_STOPF) 
  {
    uart_write("Condition5 \r\n");
    I2C->CR2 |= I2C_CR2_ACK;	  // CR2 write to clear STOPF
		I2C_transaction_end();
	}
/* Slave address matched (= Start Comm) */
  if (sr1 & I2C_SR1_ADDR)
  {	 
    uart_write("Condition6 \r\n");
		I2C_transaction_begin();
	}
/* More bytes to transmit ? */
  if ((sr1 & (I2C_SR1_TXE | I2C_SR1_BTF)) == (I2C_SR1_TXE | I2C_SR1_BTF))
  {
    uart_write("Condition7 \r\n");
		I2C->DR = I2C_byte_write();
  }
/* Byte to transmit ? */
  if (sr1 & I2C_SR1_TXE)
  {
    uart_write("Condition8 \r\n");
		I2C->DR = 0x19; //I2C_byte_write();
  }	
}

void main(void)
{

  //GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOD);

  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_WriteHigh(GPIOD, GPIO_PIN_4);

  // i2c gpio
	GPIO_Init(GPIOB,GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT); // pull up 모드도 동작함
	GPIO_Init(GPIOB,GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);
  

  // uart gpio
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);

  // UART Init
  UART1_Init(115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1,  UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

  uart_write("hello world! \r\n");

  //Initialize I2C in slave mode
 I2C_Slave_Init(I2C_ADDRESS);
 
        uint8_t InputClockFrequencyMHz= CLK_GetClockFreq()/1000000;
//  I2C_Init(I2C_SPEED, I2C_ADDRESS,I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, InputClockFrequencyMHz);
 //   I2C_ITConfig(I2C_IT_ERR, ENABLE);
 //   I2C_ITConfig(I2C_IT_EVT, ENABLE);
//    I2C_ITConfig(I2C_IT_BUF, ENABLE);
//enableInterrupts();
    // Enable interrupts globally
    __asm__("rim");

  // Loop
  

  while (1){
       delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);

    uart_write("I2C SR1 \r\n");
    UART1_SendRegisterValue("I2C->SR1", I2C->SR1);


       delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    uart_write("I2C SR2 \r\n");
    UART1_SendRegisterValue("I2C->SR2", I2C->SR2);

       delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    uart_write("I2C SR3 \r\n");
    UART1_SendRegisterValue("I2C->SR3", I2C->SR3);

      delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    delay(DELAYTIME);
    uart_write("I2C DR \r\n");
    UART1_SendRegisterValue("I2C->Dr", I2C->DR);
  }
}


void delay(uint16_t count)
{
  while (count != 0){
    count--;
  }
}

void I2C_Slave_Init(uint8_t address) {
 CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);

    // Reset I2C peripheral
    I2C->CR1 &= ~I2C_CR1_PE;   // Disable I2C before configuration

    I2C->FREQR = 16;           // Set I2C clock frequency (16 MHz)

    

    // Enable I2C interrupts
    I2C_ITConfig(I2C_IT_EVT, ENABLE);    // Event interrupts
    I2C_ITConfig(I2C_IT_ERR, ENABLE);    // Error interrupts
    I2C_ITConfig(I2C_IT_BUF, ENABLE);    // Buffer interrupts

    I2C->CR1 |= I2C_CR1_PE;    // Enable I2C
    I2C_AcknowledgeConfig(I2C_ACK_CURR);  // Enable ACK
    I2C->OARL = address << 1;  // Set 7-bit slave address
    I2C->OARH = 0;             // Enable 7-bit addressing
 
}


int uart_write(const char *str) {
    char i;
    for(i = 0; i < strlen(str); i++) {
        while(!(UART1->SR & UART1_SR_TXE)); // !Transmit data register empty
        UART1->DR = str[i];
    }
    return(i); // Bytes sent
}

// Function to send a single byte over UART
void UART1_SendByte(uint8_t byte) {
    while (!(UART1->SR & UART1_SR_TXE)); // Wait until TXE (Transmit Data Register Empty)
    UART1->DR = byte;                   // Write byte to UART data register
}

// Function to send a string over UART
void UART1_SendString(const char *str) {
    while (*str) {
        UART1_SendByte(*str++);
    }
}

// Function to send a register value in hexadecimal format
void UART1_SendRegisterValue(const char *regName, uint8_t regValue) {
    char buffer[16]; // Temporary buffer for formatted output
    UART1_SendString(regName);          // Send register name
    UART1_SendString(": 0x");           // Send "0x" prefix

    // Format the register value as a 2-digit hexadecimal string
    sprintf(buffer, "%02X", regValue);
    UART1_SendString(buffer);           // Send the formatted value

    UART1_SendString("\r\n");           // Newline for readability
}
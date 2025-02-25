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

/*
stm8flash -c stlinkv2 -p stm8s103f3 -w main.ihx

*/


#include "stm8s.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_clk.h"
#include "stm8s_exti.h"
#include "stm8s_uart1.h"
#include "stm8s_adc1.h"
#include "demo_i2c.h"
#include "demo_uart.h"

#include <string.h> 
#include <stdio.h>

/* I2C Setting */


/* GPIO Setting */
#define LED_PORT  GPIOB
#define LED_PIN   GPIO_PIN_5

#define DELAYTIME 0xF000
void delay (uint16_t count);


uint16_t ADC1_Read(void) {
    ADC1_StartConversion();
    while (ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET); // Wait for conversion to complete
    uint16_t result = ADC1_GetConversionValue(); // Read result
    ADC1_ClearFlag(ADC1_FLAG_EOC);
    return result;
}
void main(void)
{
  
  //GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOD);

  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_WriteHigh(GPIOD, GPIO_PIN_4);

  // i2c gpio



  // uart gpio
  uart1_config();

  //Initialize I2C in slave mode
  I2C_DeInit();
 I2C_Slave_Init(I2C_ADDRESS);
 
        uint8_t InputClockFrequencyMHz= CLK_GetClockFreq()/1000000;


  // ADC1 
  //CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE); 
  GPIO_Init (GPIOC, GPIO_PIN_4,  GPIO_MODE_IN_FL_NO_IT);
  uint16_t adc_value;
    ADC1_DeInit();
    ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
                ADC1_CHANNEL_2,
                ADC1_PRESSEL_FCPU_D18,
                ADC1_EXTTRIG_TIM,
                DISABLE,
                ADC1_ALIGN_RIGHT,
                ADC1_SCHMITTTRIG_ALL,
                DISABLE);
      ADC1_Cmd(ENABLE);

    __asm__("rim"); // interrupt enable


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

     adc_value = ADC1_Read(); // Read ADC value
    UART1_Send16(adc_value);
  }
}


void delay(uint16_t count)
{
  while (count != 0){
    count--;
  }
}





void I2C_IRQHandler(void) __interrupt(19) {
static u8 recv_data=0x00;
  GPIO_WriteReverse(GPIOD,GPIO_PIN_4);
    uart_write("---------------INTERRUPT ROUTRINE------------------\n");


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
        recv_data=I2C->DR;

        
        //I2C_byte_received(I2C->DR);
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

        uart_write("I2C SR1 \r\n");
        UART1_SendRegisterValue("I2C->SR1", I2C->SR1);

            uart_write("I2C SR2 \r\n");
        UART1_SendRegisterValue("I2C->SR2", I2C->SR2);

        uart_write("I2C SR3 \r\n");
        UART1_SendRegisterValue("I2C->SR3", I2C->SR3);

        uart_write("I2C DR \r\n");
        UART1_SendRegisterValue("I2C->Dr", I2C->DR);

        if(recv_data==0x13){
        I2C->DR=0x18;
        uart_write("recv success!!! \r\n");}
        else{
          I2C->DR=0x00;
        uart_write("recv fail!! \r\n");

        }
            //I2C->DR = 0x19; //I2C_byte_write();
    }	
  
}


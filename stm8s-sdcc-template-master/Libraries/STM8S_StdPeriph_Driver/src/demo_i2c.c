#include "demo_i2c.h"

static u8 u8_My_Buffer[MAX_BUFFER];
static u8 *u8_MyBuffp;
static u8 MessageBegin;

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


void I2C_Slave_Init(uint8_t address) {
    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
    GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);
    
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
   
       // Reset I2C peripheral
       I2C->CR1 &= ~I2C_CR1_PE;   // Disable I2C before configuration
   
       I2C->FREQR = 16;           // Set I2C clock frequency (16 MHz)
   
   
    
   
     I2C->CR1 |= I2C_CR1_PE;    // Enable I2C   
   I2C_AcknowledgeConfig(I2C_ACK_CURR);  // Enable ACK
   
       // Enable I2C interrupts
       I2C_ITConfig(I2C_IT_EVT, ENABLE);    // Event interrupts
       I2C_ITConfig(I2C_IT_ERR, ENABLE);    // Error interrupts
       I2C_ITConfig(I2C_IT_BUF, ENABLE);    // Buffer interrupts
   
   
      
       I2C->OARL = address << 1;  // Set 7-bit slave address
       I2C->OARH = 0;             // Enable 7-bit addressing
   
   }
   
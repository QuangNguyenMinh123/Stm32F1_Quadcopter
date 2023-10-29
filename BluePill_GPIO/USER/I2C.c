#include "I2C.h"
#include "GPIO.h"
#include "CLOCK.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define READ
#define WRITE
#define ACK
#define NACK
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static unsigned uint32_t dummy = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/
/********************************I2C1 setting**********************************/
void I2C1_Init(I2C_SPEED Speed) {
	/* PIN B6: SCL1 , PIN B7: SDA1, both as alternate function open drain*/
	GPIO_SetOutPut(IO_B6, Alternate_Open_Drain);
	GPIO_SetOutPut(IO_B7, Alternate_Open_Drain);
	/* I2C setting */
	/* I2C1 clock enable */
	if ( (RCC->APB1ENR & BIT_21) == 0 ) {
		RCC->APB1ENR |= BIT_21;
	}
	/* Generic setting: Reset I2C */
	/* I2C1 Reset */
	delay(100);
	while ( (I2C1->SR2 & BIT_1) !=0);
	I2C1->CR1 |= BIT_15;
	while ( (I2C1->SR2 & BIT_1) !=0);
	I2C1->CR1 = 0;
	if (Speed == I2C_SPEED_100)
	{
		/* Clock setting FREQ = 8MHz */
		I2C1->CR2 |= (8<<0);
		/* Speed setting */
		/* Standard mode */
		I2C1->TRISE = 9;
		I2C1->CCR |= 28;
	}
	else if (Speed == I2C_SPEED_400)
	{
		/* Clock setting FREQ = 10MHz */
		I2C1->CR2 |= (10<<0);
		/* Speed setting */
		/* Fast mode */
		I2C1->TRISE = 4;
		I2C1->CCR |= BIT_15;
		I2C1->CCR |= 50;
	}
	/* ACK enable, general call enable, enable I2C */
	I2C1->CR1 |= BIT_6 | BIT_0;
}

__inline void I2C1_Start (void)
{
/*********************** STEPS FOLLOWED  **************************************/
/*	1. Send the START condition 
	2. Wait for the SB ( Bit 0 in SR1) to set. 
	 This indicates that the start condition is generated */
/******************************************************************************/	
	if (I2C1->CR1 & BIT_9)
		I2C1->CR1 &= ~BIT_9;
	while (I2C1->CR1 & BIT_9);
	/* Enable ACK */
	I2C1->CR1 |= (1<<10);  
	/* I2C Start */
	I2C1->CR1 |= (1<<8);  
	/* Wait until start condition is successfully gernerated 	*/
	while (!(I2C1->SR1 & BIT_0));
}


__inline void I2C1_Write (uint8_t data)
{
/*********************** STEPS FOLLOWED  **************************************/
/* 	1. Wait for the TXE in SR1 reg to set. This indicates that the DR is empty
	2. Send the DATA to the DR Register
	3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of 
		LAST DATA transmission									*/
/******************************************************************************/	
	/* Wait until TxE bit is set */
	while (!(I2C1->SR1 & (1<<7)));
	I2C1->DR = data;
	/* Wait until Byte transfer is finished */
	while (!(I2C1->SR1 & (1<<2)));
}

void I2C1_CallAdress (uint8_t Address)
{
/*********************** STEPS FOLLOWED  **************************************/
/*	1. Send the Slave Address to the DR Register
	2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of
		address transmission
	3. clear the ADDR by reading the SR1 and SR2				*/
/******************************************************************************/		
	if (I2C1->CR1 & BIT_9)
		I2C1->CR1 &= ~BIT_9;
	while (I2C1->CR1 & BIT_9);
	I2C1->DR = Address;  
	/* Wait until ADDR is sent */
	while (!(I2C1->SR1 & BIT_1)); 
	/* read SR1 and SR2 to clear the ADDR bit */
	dummy = I2C1->SR1 | I2C1->SR2;
}
	
__inline void I2C1_Stop (void)
{
	/* Stop I2C */
	I2C1->CR1 |= (1<<9);  
}

void I2C1_MultiWrite(uint8_t* data, uint8_t size) {
/*********************** STEPS FOLLOWED  **************************************/
/*	1. Wait for the TXE in SR1 reg to set. This indicates that the DR is empty
	2. Keep Sending DATA to the DR Register after performing the check if the
		TXE bit is set
	3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to 
		set. This indicates the end of LAST DATA transmission		*/
/******************************************************************************/		
	/* Wait until TxE bit is set */
	while (!(I2C1->SR1 & BIT_7));
	while (size--) {
		I2C1->DR = *(data++);
		/* Wait until data is sent */
		while (!(I2C1->SR1 & BIT_7));
	}
	while (!(I2C1->SR1 & BIT_2));
}

/********************************I2C2 setting**********************************/
void I2C2_Init(I2C_SPEED Speed) {
	/* PIN B10: SCL2 , PIN B11: SDA2, both as alternate function open drain*/
	GPIO_SetOutPut(IO_B10, Alternate_Open_Drain);
	GPIO_SetOutPut(IO_B11, Alternate_Open_Drain);
	/* I2C setting */
	/* I2C2 clock enable */
	delay(100);
	if ( (RCC->APB1ENR & BIT_22) == 0 ) {
		RCC->APB1ENR |= BIT_22;
	}
	/* Generic setting: Reset I2C */
	/* I2C1 Reset */
	delay(100);
	while ( (I2C2->SR2 & BIT_1) !=0);
	I2C2->CR1 |= BIT_15;
	delay(100);
	while ( (I2C2->SR2 & BIT_1) !=0);
	I2C2->CR1 = 0;
	if (Speed == I2C_SPEED_100)
	{
		/* Clock setting FREQ = 8MHz */
		I2C2->CR2 |= (8<<0);
		/* Speed setting */
		/* Standard mode */
		I2C2->TRISE = 9;
		I2C2->CCR |= 28;
	}
	else if (Speed == I2C_SPEED_400)
	{
		/* Clock setting FREQ = 20MHz */
		I2C2->CR2 |= (20<<0);
		/* Speed setting */
		/* Fast mode */
		I2C2->TRISE = 7;
		I2C2->CCR |= BIT_15 | BIT_14;
		I2C2->CCR |= 2;
	}
	delay(100);
	/* ACK enable, general call enable, enable I2C */
	I2C2->CR1 |= BIT_0;
}

void I2C2_Start (void)
{
/*********************** STEPS FOLLOWED  **************************************/
/*	1. Send the START condition 
	2. Wait for the SB ( Bit 0 in SR1) to set. 
	 This indicates that the start condition is generated */
/******************************************************************************/	
	if (I2C2->CR1 & BIT_9)
		I2C2->CR1 &= ~BIT_9;
	while (I2C2->CR1 & BIT_9);
	/* Enable ACK */
	I2C2->CR1 |= BIT_10;  
	/* I2C Start */
	I2C2->CR1 |= BIT_8;  
	/* Wait until start condition is successfully gernerated 	*/
	while ((I2C2->SR1 & BIT_0) == 0);
}


void I2C2_Write (uint8_t data)
{
/*********************** STEPS FOLLOWED  **************************************/
/* 	1. Wait for the TXE in SR1 reg to set. This indicates that the DR is empty
	2. Send the DATA to the DR Register
	3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of 
		LAST DATA transmission									*/
/******************************************************************************/	
	/* Wait until TxE bit is set */
	while (!(I2C2->SR1 & BIT_7));
	I2C2->DR = data;
	/* Wait until Byte transfer is finished */
	while (!(I2C2->SR1 & BIT_2));
}

uint8_t I2C2_Read1Byte (void)
{
	while ((I2C2->SR1 & BIT_6) == 0) {}
	return (uint8_t)I2C2->DR;
}

void I2C2_CallAdress (uint8_t Address)
{
/*********************** STEPS FOLLOWED  **************************************/
/*	1. Send the Slave Address to the DR Register
	2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of
		address transmission
	3. clear the ADDR by reading the SR1 and SR2				*/
/******************************************************************************/		
	if (I2C2->CR1 & BIT_9)
		I2C2->CR1 &= ~BIT_9;
	while (I2C2->CR1 & BIT_9);
	I2C2->DR = Address;
	/* Wait until ADDR is sent */
	while ((I2C2->SR1 & BIT_1) == 0);
	/* read SR1 and SR2 to clear the ADDR bit */
	dummy = I2C2->SR1 | I2C2->SR2;
}
	
void I2C2_Stop (void)
{
	/* Stop I2C */
	I2C2->CR1 |= BIT_9;
	/* Diable ACK */
	I2C2->CR1 &= ~BIT_10;
}

void I2C2_MultiWrite(uint8_t* data, uint8_t size) {
/*********************** STEPS FOLLOWED  **************************************/
/*	1. Wait for the TXE in SR1 reg to set. This indicates that the DR is empty
	2. Keep Sending DATA to the DR Register after performing the check if the
		TXE bit is set
	3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to 
		set. This indicates the end of LAST DATA transmission		*/
/******************************************************************************/		
	
	while (size--) {
		/* Wait until TxE bit is set */
		while (!(I2C2->SR1 & BIT_7));
		I2C1->DR = *(data++);
		/* Wait until data is sent */
	}
	/* BTF: Byte transfer finished */
	while (!(I2C2->SR1 & BIT_2));
}

void I2C2_Read(uint8_t Adr, uint8_t* Buffer, uint8_t size) {
	uint8_t ui8Remaining = size;
	/* Call the device */
	I2C2_CallAdress(Adr);
	if (size == 1) {
		/* Disable ACK */
		I2C2->CR1 &= ~(BIT_10);
		/* Stop I2C2 */
		I2C2_Stop();
		/* Wait until RxNE is set, indicates receiving is completed */
		while ((I2C2->SR1 & BIT_6) == 0);
		/* Get the data */
		*Buffer = (uint8_t) I2C2->DR;
	} else
	if (size > 1) {
		while (ui8Remaining > 2) {
			/* Get the data */
			*(Buffer++) = I2C2_Read1Byte();
			/* ACK enable */
			ui8Remaining--;
		}
		/* Read the second last bytes */
		*(Buffer++) = I2C2_Read1Byte();
		/* Clear the ACK */
		I2C2->CR1 &= ~BIT_10;
		I2C2_Stop();
		/* Read the last byte */
		*Buffer = I2C2_Read1Byte();
	}
}

void I2C2_WriteToDevice(uint8_t DeviceId, uint8_t RegisterAddress, uint8_t Data) {
	I2C2_Start();
	I2C2_CallAdress( (uint8_t) ((DeviceId<<1) | I2C_WRITE) );
	I2C2_Write(RegisterAddress);
	I2C2_Write(Data);
	I2C2_Stop();
}

void I2C2_ReadDevice(uint8_t DeviceId, uint8_t RegisterAddress, uint8_t *buffer, uint8_t size) {
	I2C2_Start ();
	I2C2_CallAdress ( (uint8_t) ((DeviceId<<1) | I2C_WRITE) );
	I2C2_Write (RegisterAddress);
	I2C2_Start ();
	I2C2_Read ((uint8_t) ((DeviceId<<1) | I2C_READ), buffer, size);
	I2C2_Stop ();
}

void I2C2_WriteBit(uint8_t DeviceId, uint8_t RegisterAddress, uint8_t Offset, uint8_t Data) {
	uint8_t RegVal = 0U;
	I2C2_ReadDevice(DeviceId,RegisterAddress, &RegVal,1);
	RegVal = (Data != 0) ? (RegVal | (1<<Offset)) : (RegVal & ~(1<<Offset));
	I2C2_WriteToDevice(DeviceId,RegisterAddress,RegVal);
}

bool I2C2_CheckDevice(uint8_t DeviceID){
	bool bRet = FALSE;
	I2C2_Start();
	I2C2->DR = (uint8_t) (DeviceID<<1);  
	/* Wait until ADDR is sent */
	delay(MS);
	if ((I2C2->SR1 & BIT_10) == 0) {
		bRet = TRUE;
		I2C2->SR1 &= ~BIT_10;
	}
	/* read SR1 and SR2 to clear the ADDR bit */
	dummy = I2C2->SR1 | I2C2->SR2;
	I2C2_Stop();
	return bRet;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

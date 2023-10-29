#include "EX_EEPROM.h"
#include "I2C.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
bool EEPROM_CheckDevice(void) {
	bool Ret_b = FALSE;
	if (I2C2_CheckDevice(EEPROM_ADR)) {
		Ret_b = TRUE;
	}
	return Ret_b;
}

void EEPROM_Write(uint16_t Address, char* Data) {
	uint16_t Count = 0;
	I2C2_Start();
	I2C2_CallAdress(EEPROM_WRITE_ADR);
	I2C2_Write((Address & 0xff));
	I2C2_Write(Address>>8);
	while (*Data != 0) {
		Count++;
		I2C2_Write(*Data);
		Data++;
	}
	I2C2_Stop();
}

void EEPROM_Read(uint16_t Address, char* Buffer, uint8_t size) {
	uint8_t ui8Remaining = size;
	I2C2_Start();
	I2C2_CallAdress(EEPROM_WRITE_ADR);
	I2C2_Write((Address & 0xff));
	I2C2_Write(Address>>8);
	I2C2_Start();
	/* Call the device */
	I2C2_CallAdress(EEPROM_READ_ADR);
	if (size == 1) {
		/* Disable ACK */
		I2C2->CR1 &= ~(BIT_10);
		/* Stop I2C2 */
		I2C2_Stop();
		/* Wait until RxNE is set, indicates receiving is completed */
		while ((I2C2->SR1 & BIT_6) == 0);
		/* Get the data */
		*Buffer = (uint8_t) I2C2->DR;
	}else
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
/*******************************************************************************
 * EOF
 ******************************************************************************/

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
uint8_t EEPROM_CheckDevice(void) {
	uint8_t Ret_ui8 = 0;
	I2C2_Start();
	if (I2C2_CheckDevice(EEPROM_ADR)) {
		Ret_ui8 = EEPROM_ADR;
	}
	return Ret_ui8;
}

void EEPROM_Write(uint16_t Address, uint8_t* Data) {
	I2C2_Start();
	I2C2_CallAdress(EEPROM_WRITE_ADR);
	I2C2_Write(Address>>8);
	I2C2_Write((Address & 0xff));
	while (*Data != 0) {
		I2C2_Write(*Data);
		Data++;
	}
	I2C2_Stop();
}

void EEPROM_Read(uint16_t Address, uint8_t* Buffer, uint8_t size) {
	I2C2_Start();
	I2C2_CallAdress(EEPROM_WRITE_ADR);
	I2C2_Write(Address>>8);
	I2C2_Write((Address & 0xff));
	I2C2_Start();
	I2C2_CallAdress(EEPROM_READ_ADR);
	while (size--) {
		*Buffer = I2C2_Read1Byte();
		Buffer++;
	}
	I2C2_Stop();
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

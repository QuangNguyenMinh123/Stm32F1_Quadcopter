#ifndef _EX_EEPROM_H
#define _EX_EEPROM_H
#include "EX_EEPROM.h"
#include "COMMON.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EEPROM_ADR				0x50  							/* EEPROM default i2c address */
#define EEPROM_WRITE_ADR		(EEPROM_ADR<<1 | I2C_WRITE)  	/* EEPROM default i2c address */
#define EEPROM_READ_ADR			(EEPROM_ADR<<1 | I2C_READ)  	/* EEPROM default i2c address */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
bool EEPROM_CheckDevice(void);
void EEPROM_Write(uint16_t Address, char* Data);
void EEPROM_Read(uint16_t Address, char* Buffer, uint8_t size);
#endif   /* _EX_EEPROM_H */

#ifndef _I2C_H
#define _I2C_H
#include "COMMON.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef APB1PERIPH_BASE
	#define PERIPH_BASE       	((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
	#define APB1PERIPH_BASE     PERIPH_BASE
#endif

#define I2C1_BASE             	(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             	(APB1PERIPH_BASE + 0x5800)

/** 
  * @brief Inter Integrated Circuit Interface
  */

typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t OAR1;
  uint16_t  RESERVED2;
  __IO uint16_t OAR2;
  uint16_t  RESERVED3;
  __IO uint16_t DR;
  uint16_t  RESERVED4;
  __IO uint16_t SR1;
  uint16_t  RESERVED5;
  __IO uint16_t SR2;
  uint16_t  RESERVED6;
  __IO uint16_t CCR;
  uint16_t  RESERVED7;
  __IO uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)

/* I2C command and status definitions */
#define I2C_GENERAL_CALL				0x0
#define I2C_READ						1					
#define I2C_WRITE						0
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef enum{
	I2C_SPEED_100 = 0,
	I2C_SPEED_400 = 1
} I2C_SPEED;
/*******************************************************************************
 * API
 ******************************************************************************/
/* I2C1 setting */
void I2C1_Init(I2C_SPEED Speed);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);
void I2C1_CallAdress(uint8_t Adr);
void I2C1_MultiWrite(uint8_t* data, uint8_t size);
/* I2C2 setting */
void I2C2_Init(I2C_SPEED Speed);
void I2C2_Start(void);
void I2C2_Stop(void);
void I2C2_Write(uint8_t data);
uint8_t I2C2_Read1Byte(void);
void I2C2_CallAdress(uint8_t Adr);
void I2C2_MultiWrite(uint8_t* data, uint8_t size);
void I2C2_Read(uint8_t Adr, uint8_t* Buffer, uint8_t size);
bool I2C2_CheckDevice(uint8_t DeviceID);
void I2C2_WriteToDevice(uint8_t DeviceId, uint8_t RegisterAddress, uint8_t Data);
void I2C2_WriteBit(uint8_t DeviceId, uint8_t RegisterAddress, uint8_t Offset, uint8_t Data);
void I2C2_ReadDevice(uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t *buffer, uint8_t size);
#endif   /* _I2C_H */

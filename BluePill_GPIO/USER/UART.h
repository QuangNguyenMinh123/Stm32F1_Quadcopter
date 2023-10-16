#ifndef _UART_H
#define _UART_H
#include "COMMON.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
 #ifndef PERIPH_BASE
	#define PERIPH_BASE           ((uint32_t)0x40000000) 
#endif

#ifndef APB1PERIPH_BASE
	#define APB1PERIPH_BASE       (PERIPH_BASE)
#endif

#ifndef APB2PERIPH_BASE
	#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#endif
 
 #ifndef AHBPERIPH_BASE
	#define AHBPERIPH_BASE       (PERIPH_BASE + 0x20000)
#endif

 /** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter Register
  */
typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t BRR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t CR3;
  __IO uint32_t GTPR;
} USART_TypeDef;
/* USART1 Module */
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define USART1              ((USART_TypeDef *) USART1_BASE)
/* USART2 Module */
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART2              ((USART_TypeDef *) USART2_BASE)
/* USART3 Module */
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define USART3              ((USART_TypeDef *) USART3_BASE)
/******************* Define Baudrate ******************************************/
typedef enum {
	UART_BAUDRATE_2400 = 30000,
	UART_BAUDRATE_9600 = 7500,
	UART_BAUDRATE_19200 = 3750,
	UART_BAUDRATE_38400 = 1875,
	UART_BAUDRATE_57600 = 1250,
	UART_BAUDRATE_115200 = 625,
	UART_BAUDRATE_230400 = 312,
	UART_BAUDRATE_460800 = 156,
	UART_BAUDRATE_921600 = 78
} UART_BAUDRATE;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
void UART1_Init(UART_BAUDRATE Baudrate);
void UART1_sendByte(uint8_t data);
void UART1_sendStr(uint8_t* data);
void UART1_sendNum(uint32_t Val);
bool UART1_CheckMessageAvail(void);
uint8_t* UART1_GetStr(void);
#endif   /* _UART_H */

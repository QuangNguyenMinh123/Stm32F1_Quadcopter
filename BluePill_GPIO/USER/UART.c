#include "UART.h"
#include "GPIO.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SYSCLOCK					72000000U
#define TRANSMIT_COMPLETED			(USART1->SR & (1<<6))
#define TRANSMIT_NOT_COMPLETED		!(USART1->SR & (1<<6))
#define BUFFER_SIZE					128U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t USART1_Buffer[BUFFER_SIZE];
static uint8_t *Pointer;
static uint32_t ui32gDummy;
static bool bMessageAvail = FALSE;
/*******************************************************************************
 * Code
 ******************************************************************************/
static void UART1_EnaleReceiveIRQ(void) {
	/* USART_CR1 : RXNEIE enable */
	USART1->CR1 &= ~(1<<5);
	USART1->CR1 |= (1<<5);
	/* NVIC Interrupt Set Enable Register */
	NVIC_EnableIRQ(USART1_IRQ);
}
void UART1_Init(UART_BAUDRATE Baudrate) {
	Pointer = USART1_Buffer;
	/* Enable UART1 Clock RCC_APB2ENR */
	if ( (RCC->APB2ENR & (1<<14)) == 0 ) {
		RCC->APB2ENR |= (1<<14);
	}
	/* GPIO A9, A10 as UART, A9 as TX, A10 as RX */
	/* Enable PORTA clock */
	if ( (RCC->APB2ENR & (1<<2)) == 0 ) {
		RCC->APB2ENR |= 1<<2;
	}
	/* A9 as Alternate function push-pull */
	GPIOA->CRH &= ~(15<<4);
	GPIOA->CRH |= (2<<6);
	GPIOA->CRH |= (2<<4);
	/* A10 as Input doubleing */
	GPIOA->CRH &= ~(15<<8);
	GPIOA->CRH |= (1<<10);
	/* UART1 setting */
	/* Baudrate */
	USART1->BRR &= ~(65535<<0);
	USART1->BRR |= Baudrate;
	/* USART1 enable */
	if ( (USART1->CR1 & (1<<13)) == 0) {
		USART1->CR1 |= (1<<13);
	}
	USART1->CR1 &= ~(1<<12);
	USART1->CR1 &= ~(1<<10);
	/* TE, RE enable */
	if ( (USART1->CR1 & (1<<3)) == 0) {
		USART1->CR1 |= (1<<3);
	}
	if ( (USART1->CR1 & (1<<2)) == 0) {
		USART1->CR1 |= (1<<2);
	}
	/* Stop bit = 1*/
	USART1->CR2 &= ~(3<<12);
	UART1_EnaleReceiveIRQ();
}

void UART1_sendByte(uint8_t data) {
	while (TRANSMIT_NOT_COMPLETED) {}
	USART1->DR = data;	
}

void UART1_sendStr(uint8_t* data) {
	while (*data != 0) {
		UART1_sendByte(*data);
		data++;
	}
}
/* Send a uint32 number to PC via UART1 */
void UART1_sendNum(uint32_t Val) {
    uint32_t ui32Backup = Val;
    uint8_t ui8i = 1U;
    uint8_t ui8ValLen = 0U;
    if (Val == 0) {
        UART1_sendByte('0');
    } else
    {
        while (ui32Backup > 0) {
            ui8ValLen ++;
            ui32Backup /= 10;
        }
        ui32Backup = 1;
        for (;ui8i<ui8ValLen;ui8i++) {
            ui32Backup *= 10;
        }
        while (ui8ValLen>0) {
            UART1_sendByte( (uint8_t) ((Val/ui32Backup)  % 10 ) + '0' );
            ui32Backup /= 10;
            ui8ValLen --;
        }
    }
}

bool UART1_CheckMessageAvail(void) {
	return bMessageAvail;
}

uint8_t* UART1_GetStr(void) {
	bMessageAvail = FALSE;
	return USART1_Buffer;
}

RAMFUNC void USART1_IRQHandler(void) {
	*Pointer = (uint8_t) USART1->DR;
	if (*Pointer == 0) {
		/* Overrun */
		ui32gDummy = USART1->SR;
	}
	else {
		if ( *Pointer == '\r' || *Pointer == '\n' )
		{
			bMessageAvail = TRUE;
			*Pointer = 0;
			Pointer = USART1_Buffer;
		}
		else {
			Pointer ++;
		}
	}
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

#include "CLOCK.h"
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
void CLOCK_ExternCrystal72Mhz(void) {
	/* ABH prescaler = 64*/
	RCC->CFGR &= ~(15<<4);
	RCC->CFGR |= ~(12<<4);
	
	/* Configure System Clock: 72Mhz */
	/* RCC: PREDIV1SRC entry clock source */
	if ((RCC->CFGR2 & (1<<16)) == 1) {
		RCC->CFGR2 &= ~(1<<16);
	}
	/* RCC: PREDIV1 division factor */
	RCC->CFGR2 &= ~(15<<0);   /* Div 1 */
	/* RCC: PLLSRC PLL entry clock source */
	if ((RCC->CFGR & (1<<16)) == 0) {
		RCC->CFGR2 |= (1<<16);
	}
	/* RCC: PLLMUL PLL multiplication factor */
	RCC->CFGR &= ~(15<<16);
	RCC->CFGR |= (7<<16);		/* 8Mhz * 9factor = 72Mhz*/ 
	/* RCC: SW System clock switch */
	RCC->CFGR &= ~(3<<0);
	RCC->CFGR |= (2<<0);
	
	/*MCO-CFGR Configure Microcontroller clock output: 8Mhz external crystal */
	RCC->CFGR &= ~(7<<24);
	RCC->CFGR |= (6<<24);
	
	/* RCC_CFGR, configure APB1, APB2 prescaler */
	/* APB2 prescaler = 1 */
	RCC->CFGR &= ~(7<<11);
	/* APB1 prescaler = 2 */
	RCC->CFGR &= ~(7<<8);
	RCC->CFGR |= ~(4<<8);
	
	/* ABH prescaler = 1*/
	RCC->CFGR &= ~(15<<4);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

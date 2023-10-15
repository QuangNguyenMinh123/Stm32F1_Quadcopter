#include "CLOCK.h"
#include "GPIO.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TIMER1_ARR_SIZE				0xffff
#define TIMER2_ARR_SIZE				0xffff
#define TIMER3_ARR_SIZE				0xffff
#define TIMER4_ARR_SIZE				0xffff
#define TIMER5_ARR_SIZE				0xffff
#define TIMER6_ARR_SIZE				0xffff
#define TIMER7_ARR_SIZE				0xffff
#define TIMER8_ARR_SIZE				0xffff
#define TIMER9_ARR_SIZE				0xffff
#define TIMER10_ARR_SIZE			0xffff
#define TIMER11_ARR_SIZE			0xffff
#define TIMER12_ARR_SIZE			0xffff
#define TIMER13_ARR_SIZE			0xffff
#define TIMER14_ARR_SIZE			0xffff
#define SYSTICK_ARR_VALUE			9000000U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static unsigned uint32_t ui32micros = 0;
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

void CLOCK_SystickInit(void) {
	/* https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-timer--systick/systick-control-and-status-register */
	ui32micros=0U;
	/* Program the reload value */
	/* SysTick Reload Value Register */
	/* 8997620 */
	SysTick->LOAD |= (SYSTICK_ARR_VALUE - 1); /* Minus 1 because counter count down to 0 */
	SysTick->VAL   = 0UL;
	
	NVIC_SetPriority (SysTick_IRQ, (1UL << NVIC_PRIO_BITS) - 1UL);
	
	/* SysTick Control and Status Register */
	/* bit 0: enable; bit 1: IRQ enable, systick clock = 72M / 8 = 9M */
	SysTick->CTRL = 0;
	SysTick->CTRL |= BIT_0 | BIT_1;
}

/**************************SYSTICK HANDLER*************************************/
/* static void SysTick_Handler(void) {} */
void SysTick_Handler(void) {
	ui32micros += (SYSTICK_ARR_VALUE / 9) + 1; 
}

unsigned uint32_t micros(void) {
	return ( ui32micros + ((SYSTICK_ARR_VALUE - SysTick->VAL) / 9) );
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

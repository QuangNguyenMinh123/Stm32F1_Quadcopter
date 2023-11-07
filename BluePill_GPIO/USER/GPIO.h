#ifndef _GPIO_H
#define _GPIO_H
#include "COMMON.h"
#include "CLOCK.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define	LOW							0U
#define HIGH						!LOW

 #ifndef PERIPH_BASE
	#define PERIPH_BASE          	((uint32_t)0x40000000) 
#endif

#ifndef APB1PERIPH_BASE
	#define APB1PERIPH_BASE       	(PERIPH_BASE)
#endif

#ifndef APB2PERIPH_BASE
	#define APB2PERIPH_BASE       	(PERIPH_BASE + 0x10000)
#endif
 
 #ifndef AHBPERIPH_BASE
	#define AHBPERIPH_BASE       	(PERIPH_BASE + 0x20000)
#endif

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)

/*******************************ADC CONTROLLER*********************************/
/** 
  * @brief Analog to Digital Converter  
  */

typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x3C00)

#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)

#define ADC_CONVERSION_COMPLETED			ADC1->SR  & (1<<1)
#define SERVO_START							TIM4->CNT = 4999
/******************************EXTI********************************************/
/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

typedef enum {
	IO_PORTA = 0,
	IO_PORTB = 16,
	IO_PORTC = 32,
	IO_A0	= IO_PORTA + 0,			
	IO_A1	= IO_PORTA + 1,
	IO_A2	= IO_PORTA + 2,
	IO_A3	= IO_PORTA + 3,
	IO_A4	= IO_PORTA + 4,
	IO_A5	= IO_PORTA + 5,
	IO_A6	= IO_PORTA + 6,
	IO_A7	= IO_PORTA + 7,
	IO_A8	= IO_PORTA + 8,
	IO_A9	= IO_PORTA + 9,
	IO_A10	= IO_PORTA + 10,
	IO_A11	= IO_PORTA + 11,
	IO_A12	= IO_PORTA + 12,
	IO_A15	= IO_PORTA + 15,	
	IO_B0	= IO_PORTB + 0,
	IO_B1	= IO_PORTB + 1,
	IO_B5	= IO_PORTB + 5,
	IO_B6	= IO_PORTB + 6,
	IO_B7	= IO_PORTB + 7,
	IO_B8	= IO_PORTB + 8,
	IO_B9	= IO_PORTB + 9,
	IO_B10	= IO_PORTB + 10,
	IO_B11	= IO_PORTB + 11,
	IO_B12	= IO_PORTB + 12,
	IO_B13	= IO_PORTB + 13,
	IO_B14	= IO_PORTB + 14,
	IO_B15	= IO_PORTB + 15,
	IO_C13	= IO_PORTC + 13,
	IO_C14	= IO_PORTC + 14,
	IO_C15 = IO_PORTC + 15
} IO_PIN;

typedef enum {
	General_Push_Pull = 0,
	General_Open_Drain = 1,
	Alternate_Push_Pull = 2,
	Alternate_Open_Drain = 3
} OUTPUT_MODE;

typedef enum {
	Input_Pullup = 0,
	Input_Pulldown = 1,
	Input_doubleing = 2
} INPUT_MODE;

typedef enum {
	ADC_PORTA = 0,
	ADC_A0	= ADC_PORTA + 0,			
	ADC_A1	= ADC_PORTA + 1,
	ADC_A2	= ADC_PORTA + 2,
	ADC_A3	= ADC_PORTA + 3,
	ADC_A4	= ADC_PORTA + 4,
	ADC_A5	= ADC_PORTA + 5,
	ADC_A6	= ADC_PORTA + 6,
	ADC_A7	= ADC_PORTA + 7
} ADC_PIN;

typedef enum {
	FALLING_EDGE = 0,
	RISING_EDGE = 1,
	BOTH_EDGES = 2
} EDGE;

typedef struct {
  uint16_t CH1;
  uint16_t CH2;
  uint16_t CH3;
  uint16_t CH4;
  uint16_t CH5;
  uint16_t CH6;
} GPIO_PulseWidth_Type;
/*******************************************************************************
 * Global variables
 ******************************************************************************/
extern GPIO_PulseWidth_Type GPIO_PulseWidth;
/*******************************************************************************
 * API
 ******************************************************************************/
/* GPIO Output Module */
void GPIO_SetOutPut(IO_PIN PIN, OUTPUT_MODE mode);
void GPIO_PINLow(IO_PIN PIN);
void GPIO_PINHigh(IO_PIN PIN);
void GPIO_PINToggle(IO_PIN PIN);
/* GPIO Input Module */
void GPIO_SetInPut(IO_PIN PIN, INPUT_MODE MODE);
uint32_t GPIO_ReadPIN(IO_PIN PIN);
/* GPIO Analog Module */
void GPIO_SetAnalog(ADC_PIN PIN);
uint32_t GPIO_ReadAnalog(ADC_TypeDef *ADCx);
/* GPIO PWM Module */
void GPIO_SetPWM(IO_PIN PIN, uint32_t Frequency);
void GPIO_B6_PWM(uint16_t PWMValue);
void GPIO_B7_PWM(uint16_t PWMValue);
void GPIO_B8_PWM(uint16_t PWMValue);
void GPIO_B9_PWM(uint16_t PWMValue);
/* GPIO external IRQ Module */
void GPIO_SetInterrupt(IO_PIN PIN,EDGE EDGE_STATE);
/* GPIO PWM measurement */
void GPIO_SetPWMMeasurement(void);
GPIO_PulseWidth_Type* GPIO_GetPWM(void);
#endif   /* _GPIO_H */

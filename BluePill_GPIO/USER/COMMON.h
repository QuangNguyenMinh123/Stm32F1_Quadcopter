#ifndef _COMMON_H
#define _COMMON_H
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BLUEPILL
#define STMF103C8T6
/**********************************STATIC INLINE DEFINITIONS*******************/ 
#if   defined ( __CC_ARM )
  #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler */
  #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler */
  #define __STATIC_INLINE  static __inline

#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler */
  #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler */
  #define __STATIC_INLINE  static __inline

#elif defined ( __GNUC__ )
  #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler */
  #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler */
  #define __STATIC_INLINE  static inline

#elif defined ( __ICCARM__ )
  #define __ASM            __asm                                      /*!< asm keyword for IAR Compiler */
  #define __INLINE         inline                                     /*!< inline keyword for IAR Compiler. Only available in High optimization mode! */
  #define __STATIC_INLINE  static inline

#elif defined ( __TMS470__ )
  #define __ASM            __asm                                      /*!< asm keyword for TI CCS Compiler */
  #define __STATIC_INLINE  static inline

#elif defined ( __TASKING__ )
  #define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler */
  #define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler */
  #define __STATIC_INLINE  static inline

#elif defined ( __CSMC__ )
  #define __packed
  #define __ASM            _asm                                      /*!< asm keyword for COSMIC Compiler */
  #define __INLINE         inline                                    /*!< inline keyword for COSMIC Compiler. Use -pc99 on compile line */
  #define __STATIC_INLINE  static inline

#else
  #error Unknown compiler
#endif
/**********************************ADC resolution******************************/
#define ADC_RESOLUTION						4096
/**********************************RAM_FUNCTION DEFINITIONS********************/ 
  #if defined( NO_RAMFUNCS )
#define RAMFUNC
#elif defined (__ICCARM__)
#define RAMFUNC __ramfunc                                /* IAR  */
#elif defined(__CC_ARM)
#define RAMFUNC __attribute__ ((section (".ramcode")))   /* Keil MDK-ARM  */
#elif defined(__GNUC__)
#define RAMFUNC __attribute__ ((section(".ram")))    /* Sourcery G++ */
#endif

/********************************COMMON VARIABLE TYPES DEFINITIONS*************/ 
  
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */
/* Data type definitions */
#define uint8_t		unsigned char
#define uint16_t 	unsigned short int
#ifndef uint32_t
	#define uint32_t 	int
#endif
#define bool		uint8_t
#define FALSE		0U
#define TRUE		!FALSE

#define BIT_0	(1<<0)
#define BIT_1	(1<<1)
#define BIT_2	(1<<2)
#define BIT_3	(1<<3)
#define BIT_4	(1<<4)
#define BIT_5	(1<<5)
#define BIT_6	(1<<6)
#define BIT_7	(1<<7)
#define BIT_8	(1<<8)
#define BIT_9	(1<<9)
#define BIT_10	(1<<10)
#define BIT_11	(1<<11)
#define BIT_12	(1<<12)
#define BIT_13	(1<<13)
#define BIT_14	(1<<14)
#define BIT_15	(1<<15)
#define BIT_16	(1<<16)
#define BIT_17	(1<<17)
#define BIT_18	(1<<18)
#define BIT_19	(1<<19)
#define BIT_20	(1<<20)
#define BIT_21	(1<<21)
#define BIT_22	(1<<22)
#define BIT_23	(1<<23)
#define BIT_24	(1<<24)
#define BIT_25	(1<<25)
#define BIT_26	(1<<26)
#define BIT_27	(1<<27)
#define BIT_28	(1<<28)
#define BIT_29	(1<<29)
#define BIT_30	(1<<30)
#define BIT_31	(1<<31)

#define sqr(x)	x*x
/******************************************************************************/


/***************************RCC CONTROL REGISTER*******************************/
/** @addtogroup Peripheral_memory_map
  * @{
  */
  
/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;  
  __IO uint32_t AHBRSTR;
  __IO uint32_t CFGR2; 
} RCC_TypeDef;

#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          ((uint32_t)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */

#define FSMC_R_BASE           ((uint32_t)0xA0000000) /*!< FSMC registers base address */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)        /*!< PLL clock ready flag */

#ifdef STM32F10X_CL
 #define  RCC_CR_PLL2ON                       ((uint32_t)0x04000000)        /*!< PLL2 enable */
 #define  RCC_CR_PLL2RDY                      ((uint32_t)0x08000000)        /*!< PLL2 clock ready flag */
 #define  RCC_CR_PLL3ON                       ((uint32_t)0x10000000)        /*!< PLL3 enable */
 #define  RCC_CR_PLL3RDY                      ((uint32_t)0x20000000)        /*!< PLL3 clock ready flag */
#endif /* STM32F10X_CL */

/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800)        /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define  RCC_CFGR_ADCPRE                     ((uint32_t)0x0000C000)        /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_CFGR_ADCPRE_0                   ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  RCC_CFGR_ADCPRE_1                   ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  RCC_CFGR_ADCPRE_DIV2                ((uint32_t)0x00000000)        /*!< PCLK2 divided by 2 */
#define  RCC_CFGR_ADCPRE_DIV4                ((uint32_t)0x00004000)        /*!< PCLK2 divided by 4 */
#define  RCC_CFGR_ADCPRE_DIV6                ((uint32_t)0x00008000)        /*!< PCLK2 divided by 6 */
#define  RCC_CFGR_ADCPRE_DIV8                ((uint32_t)0x0000C000)        /*!< PCLK2 divided by 8 */

#define  RCC_CFGR_PLLSRC                     ((uint32_t)0x00010000)        /*!< PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE                   ((uint32_t)0x00020000)        /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMULL                    ((uint32_t)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((uint32_t)0x00200000)        /*!< Bit 3 */

#ifdef STM32F10X_CL
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock * 4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock * 5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock * 6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock * 7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock * 8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock * 9 */
 #define  RCC_CFGR_PLLMULL6_5                ((uint32_t)0x00340000)        /*!< PLL input clock * 6.5 */
 
 #define  RCC_CFGR_OTGFSPRE                  ((uint32_t)0x00400000)        /*!< USB OTG FS prescaler */
 
/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x0F000000)        /*!< MCO[3:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */
 #define  RCC_CFGR_MCO_3                     ((uint32_t)0x08000000)        /*!< Bit 3 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLLCLK_Div2           ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
 #define  RCC_CFGR_MCO_PLL2CLK               ((uint32_t)0x08000000)        /*!< PLL2 clock selected as MCO source*/
 #define  RCC_CFGR_MCO_PLL3CLK_Div2          ((uint32_t)0x09000000)        /*!< PLL3 clock divided by 2 selected as MCO source*/
 #define  RCC_CFGR_MCO_Ext_HSE               ((uint32_t)0x0A000000)        /*!< XT1 external 3-25 MHz oscillator clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLL3CLK               ((uint32_t)0x0B000000)        /*!< PLL3 clock selected as MCO source */
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((uint32_t)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#else
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_HSE                ((uint32_t)0x00010000)        /*!< HSE clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_HSE              ((uint32_t)0x00000000)        /*!< HSE clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_HSE_Div2         ((uint32_t)0x00020000)        /*!< HSE clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((uint32_t)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((uint32_t)0x00380000)        /*!< PLL input clock*16 */
 #define  RCC_CFGR_USBPRE                    ((uint32_t)0x00400000)        /*!< USB Device prescaler */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#endif /* STM32F10X_CL */

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)        /*!< LSI Ready Interrupt flag */
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)        /*!< LSE Ready Interrupt flag */
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)        /*!< HSI Ready Interrupt flag */
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)        /*!< HSE Ready Interrupt flag */
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)        /*!< PLL Ready Interrupt flag */
#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)        /*!< Clock Security System Interrupt flag */
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)        /*!< LSI Ready Interrupt Enable */
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)        /*!< LSE Ready Interrupt Enable */
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)        /*!< HSI Ready Interrupt Enable */
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)        /*!< HSE Ready Interrupt Enable */
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)        /*!< PLL Ready Interrupt Enable */
#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)        /*!< LSI Ready Interrupt Clear */
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)        /*!< LSE Ready Interrupt Clear */
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)        /*!< HSI Ready Interrupt Clear */
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)        /*!< HSE Ready Interrupt Clear */
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)        /*!< PLL Ready Interrupt Clear */
#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)        /*!< Clock Security System Interrupt Clear */

/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IO uint32_t ISER[1U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[31U];
  __IO uint32_t ICER[1U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[31U];
  __IO uint32_t ISPR[1U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[31U];
  __IO uint32_t ICPR[1U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  __IO uint32_t IP[8U];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register */
}  NVIC_Type;

/******************  Bit definition for NVIC_ISER register  *******************/
#define  NVIC_ISER_SETENA                    ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set enable bits */
#define  NVIC_ISER_SETENA_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISER_SETENA_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISER_SETENA_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISER_SETENA_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISER_SETENA_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISER_SETENA_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISER_SETENA_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISER_SETENA_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISER_SETENA_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISER_SETENA_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISER_SETENA_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISER_SETENA_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISER_SETENA_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISER_SETENA_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISER_SETENA_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISER_SETENA_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISER_SETENA_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISER_SETENA_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISER_SETENA_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISER_SETENA_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISER_SETENA_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISER_SETENA_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISER_SETENA_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISER_SETENA_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISER_SETENA_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISER_SETENA_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISER_SETENA_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISER_SETENA_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISER_SETENA_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISER_SETENA_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISER_SETENA_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISER_SETENA_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICER register  *******************/
#define  NVIC_ICER_CLRENA                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-enable bits */
#define  NVIC_ICER_CLRENA_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICER_CLRENA_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICER_CLRENA_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICER_CLRENA_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICER_CLRENA_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICER_CLRENA_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICER_CLRENA_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICER_CLRENA_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICER_CLRENA_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICER_CLRENA_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICER_CLRENA_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICER_CLRENA_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICER_CLRENA_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICER_CLRENA_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICER_CLRENA_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICER_CLRENA_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICER_CLRENA_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICER_CLRENA_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICER_CLRENA_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICER_CLRENA_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICER_CLRENA_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICER_CLRENA_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICER_CLRENA_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICER_CLRENA_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICER_CLRENA_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICER_CLRENA_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICER_CLRENA_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICER_CLRENA_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICER_CLRENA_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICER_CLRENA_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICER_CLRENA_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICER_CLRENA_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ISPR register  *******************/
#define  NVIC_ISPR_SETPEND                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set-pending bits */
#define  NVIC_ISPR_SETPEND_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISPR_SETPEND_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISPR_SETPEND_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISPR_SETPEND_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISPR_SETPEND_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISPR_SETPEND_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISPR_SETPEND_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISPR_SETPEND_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISPR_SETPEND_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISPR_SETPEND_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISPR_SETPEND_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISPR_SETPEND_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISPR_SETPEND_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISPR_SETPEND_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISPR_SETPEND_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISPR_SETPEND_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISPR_SETPEND_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISPR_SETPEND_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISPR_SETPEND_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISPR_SETPEND_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISPR_SETPEND_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISPR_SETPEND_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISPR_SETPEND_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISPR_SETPEND_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISPR_SETPEND_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISPR_SETPEND_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISPR_SETPEND_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISPR_SETPEND_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISPR_SETPEND_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISPR_SETPEND_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISPR_SETPEND_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISPR_SETPEND_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICPR register  *******************/
#define  NVIC_ICPR_CLRPEND                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-pending bits */
#define  NVIC_ICPR_CLRPEND_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICPR_CLRPEND_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICPR_CLRPEND_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICPR_CLRPEND_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICPR_CLRPEND_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICPR_CLRPEND_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICPR_CLRPEND_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICPR_CLRPEND_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICPR_CLRPEND_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICPR_CLRPEND_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICPR_CLRPEND_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICPR_CLRPEND_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICPR_CLRPEND_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICPR_CLRPEND_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICPR_CLRPEND_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICPR_CLRPEND_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICPR_CLRPEND_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICPR_CLRPEND_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICPR_CLRPEND_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICPR_CLRPEND_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICPR_CLRPEND_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICPR_CLRPEND_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICPR_CLRPEND_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICPR_CLRPEND_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICPR_CLRPEND_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICPR_CLRPEND_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICPR_CLRPEND_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICPR_CLRPEND_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICPR_CLRPEND_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICPR_CLRPEND_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICPR_CLRPEND_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICPR_CLRPEND_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_IABR register  *******************/
#define  NVIC_IABR_ACTIVE                    ((uint32_t)0xFFFFFFFF)        /*!< Interrupt active flags */
#define  NVIC_IABR_ACTIVE_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_IABR_ACTIVE_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_IABR_ACTIVE_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_IABR_ACTIVE_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_IABR_ACTIVE_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_IABR_ACTIVE_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_IABR_ACTIVE_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_IABR_ACTIVE_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_IABR_ACTIVE_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_IABR_ACTIVE_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_IABR_ACTIVE_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_IABR_ACTIVE_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_IABR_ACTIVE_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_IABR_ACTIVE_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_IABR_ACTIVE_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_IABR_ACTIVE_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_IABR_ACTIVE_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_IABR_ACTIVE_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_IABR_ACTIVE_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_IABR_ACTIVE_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_IABR_ACTIVE_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_IABR_ACTIVE_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_IABR_ACTIVE_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_IABR_ACTIVE_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_IABR_ACTIVE_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_IABR_ACTIVE_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_IABR_ACTIVE_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_IABR_ACTIVE_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_IABR_ACTIVE_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_IABR_ACTIVE_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_IABR_ACTIVE_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_IABR_ACTIVE_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_PRI0 register  *******************/
#define  NVIC_IPR0_PRI_0                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 0 */
#define  NVIC_IPR0_PRI_1                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 1 */
#define  NVIC_IPR0_PRI_2                     ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 2 */
#define  NVIC_IPR0_PRI_3                     ((uint32_t)0xFF000000)        /*!< Priority of interrupt 3 */

/******************  Bit definition for NVIC_PRI1 register  *******************/
#define  NVIC_IPR1_PRI_4                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 4 */
#define  NVIC_IPR1_PRI_5                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 5 */
#define  NVIC_IPR1_PRI_6                     ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 6 */
#define  NVIC_IPR1_PRI_7                     ((uint32_t)0xFF000000)        /*!< Priority of interrupt 7 */

/******************  Bit definition for NVIC_PRI2 register  *******************/
#define  NVIC_IPR2_PRI_8                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 8 */
#define  NVIC_IPR2_PRI_9                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 9 */
#define  NVIC_IPR2_PRI_10                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 10 */
#define  NVIC_IPR2_PRI_11                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 11 */

/******************  Bit definition for NVIC_PRI3 register  *******************/
#define  NVIC_IPR3_PRI_12                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 12 */
#define  NVIC_IPR3_PRI_13                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 13 */
#define  NVIC_IPR3_PRI_14                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 14 */
#define  NVIC_IPR3_PRI_15                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 15 */

/******************  Bit definition for NVIC_PRI4 register  *******************/
#define  NVIC_IPR4_PRI_16                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 16 */
#define  NVIC_IPR4_PRI_17                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 17 */
#define  NVIC_IPR4_PRI_18                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 18 */
#define  NVIC_IPR4_PRI_19                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 19 */

/******************  Bit definition for NVIC_PRI5 register  *******************/
#define  NVIC_IPR5_PRI_20                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 20 */
#define  NVIC_IPR5_PRI_21                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 21 */
#define  NVIC_IPR5_PRI_22                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 22 */
#define  NVIC_IPR5_PRI_23                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 23 */

/******************  Bit definition for NVIC_PRI6 register  *******************/
#define  NVIC_IPR6_PRI_24                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 24 */
#define  NVIC_IPR6_PRI_25                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 25 */
#define  NVIC_IPR6_PRI_26                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 26 */
#define  NVIC_IPR6_PRI_27                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 27 */

/******************  Bit definition for NVIC_PRI7 register  *******************/
#define  NVIC_IPR7_PRI_28                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 28 */
#define  NVIC_IPR7_PRI_29                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 29 */
#define  NVIC_IPR7_PRI_30                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 30 */
#define  NVIC_IPR7_PRI_31                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 31 */

#define NVIC_PRIO_BITS          4U
/******************************************************************************/
/*                                                                            */
/*                  			SYSTICK CONTROLLER		                      */
/*                                                                            */
/******************************************************************************/

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __IO uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IO uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IO uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __I  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;
/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos         16U                                            /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos          2U                                            /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk         (1UL << SysTick_CTRL_CLKSOURCE_Pos)            /*!< SysTick CTRL: CLKSOURCE Mask */

#define SysTick_CTRL_TICKINT_Pos            1U                                            /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk           (1UL << SysTick_CTRL_TICKINT_Pos)              /*!< SysTick CTRL: TICKINT Mask */

#define SysTick_CTRL_ENABLE_Pos             0U                                            /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1UL /*<< SysTick_CTRL_ENABLE_Pos*/)           /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0U                                            /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/)    /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos             0U                                            /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/)    /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31U                                            /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)               /*!< SysTick CALIB: NOREF Mask */

#define SysTick_CALIB_SKEW_Pos             30U                                            /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)                /*!< SysTick CALIB: SKEW Mask */

#define SysTick_CALIB_TENMS_Pos             0U                                            /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/)    /*!< SysTick CALIB: TENMS Mask */

/*@} end of group CMSIS_SysTick */

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __I  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IO uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IO uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IO uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IO uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IO uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IO uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IO uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IO uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IO uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IO uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IO uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IO uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IO uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __I  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __I  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __I  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __I  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __I  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __IO uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER_Pos          24U                                            /*!< SCB CPUID: IMPLEMENTER Position */
#define SCB_CPUID_IMPLEMENTER_Msk          (0xFFUL << SCB_CPUID_IMPLEMENTER_Pos)          /*!< SCB CPUID: IMPLEMENTER Mask */

#define SCB_CPUID_VARIANT_Pos              20U                                            /*!< SCB CPUID: VARIANT Position */
#define SCB_CPUID_VARIANT_Msk              (0xFUL << SCB_CPUID_VARIANT_Pos)               /*!< SCB CPUID: VARIANT Mask */

#define SCB_CPUID_ARCHITECTURE_Pos         16U                                            /*!< SCB CPUID: ARCHITECTURE Position */
#define SCB_CPUID_ARCHITECTURE_Msk         (0xFUL << SCB_CPUID_ARCHITECTURE_Pos)          /*!< SCB CPUID: ARCHITECTURE Mask */

#define SCB_CPUID_PARTNO_Pos                4U                                            /*!< SCB CPUID: PARTNO Position */
#define SCB_CPUID_PARTNO_Msk               (0xFFFUL << SCB_CPUID_PARTNO_Pos)              /*!< SCB CPUID: PARTNO Mask */

#define SCB_CPUID_REVISION_Pos              0U                                            /*!< SCB CPUID: REVISION Position */
#define SCB_CPUID_REVISION_Msk             (0xFUL /*<< SCB_CPUID_REVISION_Pos*/)          /*!< SCB CPUID: REVISION Mask */

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_NMIPENDSET_Pos            31U                                            /*!< SCB ICSR: NMIPENDSET Position */
#define SCB_ICSR_NMIPENDSET_Msk            (1UL << SCB_ICSR_NMIPENDSET_Pos)               /*!< SCB ICSR: NMIPENDSET Mask */

#define SCB_ICSR_PENDSVSET_Pos             28U                                            /*!< SCB ICSR: PENDSVSET Position */
#define SCB_ICSR_PENDSVSET_Msk             (1UL << SCB_ICSR_PENDSVSET_Pos)                /*!< SCB ICSR: PENDSVSET Mask */

#define SCB_ICSR_PENDSVCLR_Pos             27U                                            /*!< SCB ICSR: PENDSVCLR Position */
#define SCB_ICSR_PENDSVCLR_Msk             (1UL << SCB_ICSR_PENDSVCLR_Pos)                /*!< SCB ICSR: PENDSVCLR Mask */

#define SCB_ICSR_PENDSTSET_Pos             26U                                            /*!< SCB ICSR: PENDSTSET Position */
#define SCB_ICSR_PENDSTSET_Msk             (1UL << SCB_ICSR_PENDSTSET_Pos)                /*!< SCB ICSR: PENDSTSET Mask */

#define SCB_ICSR_PENDSTCLR_Pos             25U                                            /*!< SCB ICSR: PENDSTCLR Position */
#define SCB_ICSR_PENDSTCLR_Msk             (1UL << SCB_ICSR_PENDSTCLR_Pos)                /*!< SCB ICSR: PENDSTCLR Mask */

#define SCB_ICSR_ISRPREEMPT_Pos            23U                                            /*!< SCB ICSR: ISRPREEMPT Position */
#define SCB_ICSR_ISRPREEMPT_Msk            (1UL << SCB_ICSR_ISRPREEMPT_Pos)               /*!< SCB ICSR: ISRPREEMPT Mask */

#define SCB_ICSR_ISRPENDING_Pos            22U                                            /*!< SCB ICSR: ISRPENDING Position */
#define SCB_ICSR_ISRPENDING_Msk            (1UL << SCB_ICSR_ISRPENDING_Pos)               /*!< SCB ICSR: ISRPENDING Mask */

#define SCB_ICSR_VECTPENDING_Pos           12U                                            /*!< SCB ICSR: VECTPENDING Position */
#define SCB_ICSR_VECTPENDING_Msk           (0x1FFUL << SCB_ICSR_VECTPENDING_Pos)          /*!< SCB ICSR: VECTPENDING Mask */

#define SCB_ICSR_RETTOBASE_Pos             11U                                            /*!< SCB ICSR: RETTOBASE Position */
#define SCB_ICSR_RETTOBASE_Msk             (1UL << SCB_ICSR_RETTOBASE_Pos)                /*!< SCB ICSR: RETTOBASE Mask */

#define SCB_ICSR_VECTACTIVE_Pos             0U                                            /*!< SCB ICSR: VECTACTIVE Position */
#define SCB_ICSR_VECTACTIVE_Msk            (0x1FFUL /*<< SCB_ICSR_VECTACTIVE_Pos*/)       /*!< SCB ICSR: VECTACTIVE Mask */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_VECTKEYSTAT_Pos          16U                                            /*!< SCB AIRCR: VECTKEYSTAT Position */
#define SCB_AIRCR_VECTKEYSTAT_Msk          (0xFFFFUL << SCB_AIRCR_VECTKEYSTAT_Pos)        /*!< SCB AIRCR: VECTKEYSTAT Mask */

#define SCB_AIRCR_ENDIANESS_Pos            15U                                            /*!< SCB AIRCR: ENDIANESS Position */
#define SCB_AIRCR_ENDIANESS_Msk            (1UL << SCB_AIRCR_ENDIANESS_Pos)               /*!< SCB AIRCR: ENDIANESS Mask */

#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos           2U                                            /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */

#define SCB_AIRCR_VECTCLRACTIVE_Pos         1U                                            /*!< SCB AIRCR: VECTCLRACTIVE Position */
#define SCB_AIRCR_VECTCLRACTIVE_Msk        (1UL << SCB_AIRCR_VECTCLRACTIVE_Pos)           /*!< SCB AIRCR: VECTCLRACTIVE Mask */

#define SCB_AIRCR_VECTRESET_Pos             0U                                            /*!< SCB AIRCR: VECTRESET Position */
#define SCB_AIRCR_VECTRESET_Msk            (1UL /*<< SCB_AIRCR_VECTRESET_Pos*/)           /*!< SCB AIRCR: VECTRESET Mask */

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND_Pos               4U                                            /*!< SCB SCR: SEVONPEND Position */
#define SCB_SCR_SEVONPEND_Msk              (1UL << SCB_SCR_SEVONPEND_Pos)                 /*!< SCB SCR: SEVONPEND Mask */

#define SCB_SCR_SLEEPDEEP_Pos               2U                                            /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk              (1UL << SCB_SCR_SLEEPDEEP_Pos)                 /*!< SCB SCR: SLEEPDEEP Mask */

#define SCB_SCR_SLEEPONEXIT_Pos             1U                                            /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk            (1UL << SCB_SCR_SLEEPONEXIT_Pos)               /*!< SCB SCR: SLEEPONEXIT Mask */

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_STKALIGN_Pos                9U                                            /*!< SCB CCR: STKALIGN Position */
#define SCB_CCR_STKALIGN_Msk               (1UL << SCB_CCR_STKALIGN_Pos)                  /*!< SCB CCR: STKALIGN Mask */

#define SCB_CCR_BFHFNMIGN_Pos               8U                                            /*!< SCB CCR: BFHFNMIGN Position */
#define SCB_CCR_BFHFNMIGN_Msk              (1UL << SCB_CCR_BFHFNMIGN_Pos)                 /*!< SCB CCR: BFHFNMIGN Mask */

#define SCB_CCR_DIV_0_TRP_Pos               4U                                            /*!< SCB CCR: DIV_0_TRP Position */
#define SCB_CCR_DIV_0_TRP_Msk              (1UL << SCB_CCR_DIV_0_TRP_Pos)                 /*!< SCB CCR: DIV_0_TRP Mask */

#define SCB_CCR_UNALIGN_TRP_Pos             3U                                            /*!< SCB CCR: UNALIGN_TRP Position */
#define SCB_CCR_UNALIGN_TRP_Msk            (1UL << SCB_CCR_UNALIGN_TRP_Pos)               /*!< SCB CCR: UNALIGN_TRP Mask */

#define SCB_CCR_USERSETMPEND_Pos            1U                                            /*!< SCB CCR: USERSETMPEND Position */
#define SCB_CCR_USERSETMPEND_Msk           (1UL << SCB_CCR_USERSETMPEND_Pos)              /*!< SCB CCR: USERSETMPEND Mask */

#define SCB_CCR_NONBASETHRDENA_Pos          0U                                            /*!< SCB CCR: NONBASETHRDENA Position */
#define SCB_CCR_NONBASETHRDENA_Msk         (1UL /*<< SCB_CCR_NONBASETHRDENA_Pos*/)        /*!< SCB CCR: NONBASETHRDENA Mask */

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_USGFAULTENA_Pos          18U                                            /*!< SCB SHCSR: USGFAULTENA Position */
#define SCB_SHCSR_USGFAULTENA_Msk          (1UL << SCB_SHCSR_USGFAULTENA_Pos)             /*!< SCB SHCSR: USGFAULTENA Mask */

#define SCB_SHCSR_BUSFAULTENA_Pos          17U                                            /*!< SCB SHCSR: BUSFAULTENA Position */
#define SCB_SHCSR_BUSFAULTENA_Msk          (1UL << SCB_SHCSR_BUSFAULTENA_Pos)             /*!< SCB SHCSR: BUSFAULTENA Mask */

#define SCB_SHCSR_MEMFAULTENA_Pos          16U                                            /*!< SCB SHCSR: MEMFAULTENA Position */
#define SCB_SHCSR_MEMFAULTENA_Msk          (1UL << SCB_SHCSR_MEMFAULTENA_Pos)             /*!< SCB SHCSR: MEMFAULTENA Mask */

#define SCB_SHCSR_SVCALLPENDED_Pos         15U                                            /*!< SCB SHCSR: SVCALLPENDED Position */
#define SCB_SHCSR_SVCALLPENDED_Msk         (1UL << SCB_SHCSR_SVCALLPENDED_Pos)            /*!< SCB SHCSR: SVCALLPENDED Mask */

#define SCB_SHCSR_BUSFAULTPENDED_Pos       14U                                            /*!< SCB SHCSR: BUSFAULTPENDED Position */
#define SCB_SHCSR_BUSFAULTPENDED_Msk       (1UL << SCB_SHCSR_BUSFAULTPENDED_Pos)          /*!< SCB SHCSR: BUSFAULTPENDED Mask */

#define SCB_SHCSR_MEMFAULTPENDED_Pos       13U                                            /*!< SCB SHCSR: MEMFAULTPENDED Position */
#define SCB_SHCSR_MEMFAULTPENDED_Msk       (1UL << SCB_SHCSR_MEMFAULTPENDED_Pos)          /*!< SCB SHCSR: MEMFAULTPENDED Mask */

#define SCB_SHCSR_USGFAULTPENDED_Pos       12U                                            /*!< SCB SHCSR: USGFAULTPENDED Position */
#define SCB_SHCSR_USGFAULTPENDED_Msk       (1UL << SCB_SHCSR_USGFAULTPENDED_Pos)          /*!< SCB SHCSR: USGFAULTPENDED Mask */

#define SCB_SHCSR_SYSTICKACT_Pos           11U                                            /*!< SCB SHCSR: SYSTICKACT Position */
#define SCB_SHCSR_SYSTICKACT_Msk           (1UL << SCB_SHCSR_SYSTICKACT_Pos)              /*!< SCB SHCSR: SYSTICKACT Mask */

#define SCB_SHCSR_PENDSVACT_Pos            10U                                            /*!< SCB SHCSR: PENDSVACT Position */
#define SCB_SHCSR_PENDSVACT_Msk            (1UL << SCB_SHCSR_PENDSVACT_Pos)               /*!< SCB SHCSR: PENDSVACT Mask */

#define SCB_SHCSR_MONITORACT_Pos            8U                                            /*!< SCB SHCSR: MONITORACT Position */
#define SCB_SHCSR_MONITORACT_Msk           (1UL << SCB_SHCSR_MONITORACT_Pos)              /*!< SCB SHCSR: MONITORACT Mask */

#define SCB_SHCSR_SVCALLACT_Pos             7U                                            /*!< SCB SHCSR: SVCALLACT Position */
#define SCB_SHCSR_SVCALLACT_Msk            (1UL << SCB_SHCSR_SVCALLACT_Pos)               /*!< SCB SHCSR: SVCALLACT Mask */

#define SCB_SHCSR_USGFAULTACT_Pos           3U                                            /*!< SCB SHCSR: USGFAULTACT Position */
#define SCB_SHCSR_USGFAULTACT_Msk          (1UL << SCB_SHCSR_USGFAULTACT_Pos)             /*!< SCB SHCSR: USGFAULTACT Mask */

#define SCB_SHCSR_BUSFAULTACT_Pos           1U                                            /*!< SCB SHCSR: BUSFAULTACT Position */
#define SCB_SHCSR_BUSFAULTACT_Msk          (1UL << SCB_SHCSR_BUSFAULTACT_Pos)             /*!< SCB SHCSR: BUSFAULTACT Mask */

#define SCB_SHCSR_MEMFAULTACT_Pos           0U                                            /*!< SCB SHCSR: MEMFAULTACT Position */
#define SCB_SHCSR_MEMFAULTACT_Msk          (1UL /*<< SCB_SHCSR_MEMFAULTACT_Pos*/)         /*!< SCB SHCSR: MEMFAULTACT Mask */

/* SCB Configurable Fault Status Register Definitions */
#define SCB_CFSR_USGFAULTSR_Pos            16U                                            /*!< SCB CFSR: Usage Fault Status Register Position */
#define SCB_CFSR_USGFAULTSR_Msk            (0xFFFFUL << SCB_CFSR_USGFAULTSR_Pos)          /*!< SCB CFSR: Usage Fault Status Register Mask */

#define SCB_CFSR_BUSFAULTSR_Pos             8U                                            /*!< SCB CFSR: Bus Fault Status Register Position */
#define SCB_CFSR_BUSFAULTSR_Msk            (0xFFUL << SCB_CFSR_BUSFAULTSR_Pos)            /*!< SCB CFSR: Bus Fault Status Register Mask */

#define SCB_CFSR_MEMFAULTSR_Pos             0U                                            /*!< SCB CFSR: Memory Manage Fault Status Register Position */
#define SCB_CFSR_MEMFAULTSR_Msk            (0xFFUL /*<< SCB_CFSR_MEMFAULTSR_Pos*/)        /*!< SCB CFSR: Memory Manage Fault Status Register Mask */

/* SCB Hard Fault Status Register Definitions */
#define SCB_HFSR_DEBUGEVT_Pos              31U                                            /*!< SCB HFSR: DEBUGEVT Position */
#define SCB_HFSR_DEBUGEVT_Msk              (1UL << SCB_HFSR_DEBUGEVT_Pos)                 /*!< SCB HFSR: DEBUGEVT Mask */

#define SCB_HFSR_FORCED_Pos                30U                                            /*!< SCB HFSR: FORCED Position */
#define SCB_HFSR_FORCED_Msk                (1UL << SCB_HFSR_FORCED_Pos)                   /*!< SCB HFSR: FORCED Mask */

#define SCB_HFSR_VECTTBL_Pos                1U                                            /*!< SCB HFSR: VECTTBL Position */
#define SCB_HFSR_VECTTBL_Msk               (1UL << SCB_HFSR_VECTTBL_Pos)                  /*!< SCB HFSR: VECTTBL Mask */

/* SCB Debug Fault Status Register Definitions */
#define SCB_DFSR_EXTERNAL_Pos               4U                                            /*!< SCB DFSR: EXTERNAL Position */
#define SCB_DFSR_EXTERNAL_Msk              (1UL << SCB_DFSR_EXTERNAL_Pos)                 /*!< SCB DFSR: EXTERNAL Mask */

#define SCB_DFSR_VCATCH_Pos                 3U                                            /*!< SCB DFSR: VCATCH Position */
#define SCB_DFSR_VCATCH_Msk                (1UL << SCB_DFSR_VCATCH_Pos)                   /*!< SCB DFSR: VCATCH Mask */

#define SCB_DFSR_DWTTRAP_Pos                2U                                            /*!< SCB DFSR: DWTTRAP Position */
#define SCB_DFSR_DWTTRAP_Msk               (1UL << SCB_DFSR_DWTTRAP_Pos)                  /*!< SCB DFSR: DWTTRAP Mask */

#define SCB_DFSR_BKPT_Pos                   1U                                            /*!< SCB DFSR: BKPT Position */
#define SCB_DFSR_BKPT_Msk                  (1UL << SCB_DFSR_BKPT_Pos)                     /*!< SCB DFSR: BKPT Mask */

#define SCB_DFSR_HALTED_Pos                 0U                                            /*!< SCB DFSR: HALTED Position */
#define SCB_DFSR_HALTED_Msk                (1UL /*<< SCB_DFSR_HALTED_Pos*/)               /*!< SCB DFSR: HALTED Mask */

/*@} end of group CMSIS_SCB */

/*                System Control      		 */

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */                  /*!< System Control Block Base Address */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */

#define MS							3120U
#define SEC							3129597U
#define DEG_TO_RAD					0.01745329251
#define RAD_TO_DEG					57.2957795131
extern const double pi;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef enum {
	SysTick_IRQ = -1,
	WWDG_IRQ,
	PVD_IRQ,
	TAMPER_IRQ,
	RTC_IRQ,
	FLASH_IRQ,
	RCC_IRQ,
	EXTI0_IRQ,
	EXTI1_IRQ,
	EXTI2_IRQ,
	EXTI3_IRQ,
	EXTI4_IRQ,
	DMA1_Channel1_IRQ,
	DMA1_Channel2_IRQ,
	DMA1_Channel3_IRQ,
	DMA1_Channel4_IRQ,
	DMA1_Channel5_IRQ,
	DMA1_Channel6_IRQ,
	DMA1_Channel7_IRQ,
	ADC1_2_IRQ,
	USB_HP_CAN1_TX_IRQ,
	USB_LP_CAN1_RX0_IRQ,
	CAN1_RX1_IRQ,
	CAN1_SCE_IRQ,
	EXTI9_5_IRQ,
	TIM1_BRK_IRQ,
	TIM1_UP_IRQ,
	TIM1_TRG_COM_IRQ,
	TIM1_CC_IRQ,
	TIM2_IRQ,
	TIM3_IRQ,
	TIM4_IRQ,
	I2C1_EV_IRQ,
	I2C1_ER_IRQ,      
	I2C2_EV_IRQ,
	I2C2_ER_IRQ,
	SPI1_IRQ,
	SPI2_IRQ,
	USART1_IRQ,
	USART2_IRQ,
	USART3_IRQ,
	EXTI15_10_IRQ,
	RTCAlarm_IRQ,
	USBWakeUp_IRQ
} IRQType;
/*******************************************************************************
 * API
 ******************************************************************************/
void delay(uint32_t delayTime);
void NVIC_EnableIRQ(IRQType IRQn);
void NVIC_SetPriority(IRQType IRQn, uint32_t priority);
void NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
double map(double x, double in_min, double in_max, double out_min, double out_max);
double DegreeToRadian(double value);
double RadianToDegree(double value);
#endif   /* _COMMON_H */

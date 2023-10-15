#include "COMMON.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#define SYSTICK_MAX_RANGE		0x00FFFFFFUL
/*******************************************************************************
 * Variables
 ******************************************************************************/
const double pi = 3.14159265;
/*******************************************************************************
 * Code
 ******************************************************************************/

void delay(uint32_t delayTime) {
	uint32_t ui32Cnt = 0U;
	for (; ui32Cnt< delayTime; ui32Cnt++) {
		__asm("nop");                                                                                                                               	}
}

uint32_t map(uint32_t MapValue, uint32_t InputLow, uint32_t InputHigh, uint32_t OutputLow, uint32_t OutputHigh) {
	uint32_t a = (OutputHigh - OutputLow);
	uint32_t b = (InputHigh - InputLow) ;
	uint32_t z = MapValue * a;
	return z / b;
}

void NVIC_EnableIRQ(IRQType IRQn)
{
  NVIC->ISER[(((uint32_t)(uint32_t)IRQn) >> 5)] = (uint32_t)(1 << (((uint32_t)(uint32_t)IRQn) & 0x1F));
}

void NVIC_SetPriority(IRQType IRQn, uint32_t priority)
{
  if ((uint32_t)(IRQn) < 0)
  {
    SCB->SHP[(((uint32_t)(uint32_t)IRQn) & 0xF)-4] = (uint8_t)((priority << (8 - NVIC_PRIO_BITS)) & (uint32_t)0xFF);
  }
  else
  {
    NVIC->IP[((uint32_t)(uint32_t)IRQn)]               = (uint8_t)((priority << (8U - NVIC_PRIO_BITS)) & (uint32_t)0xFF);
  }
}

double DegreeToRadian(double value) {
	double fRet = 0.0;
	fRet = pi *value / ((double)180);
	return fRet;
}

double RadianToDegree(double value) {
	double fRet = 0.0;
	fRet = ((double)180) * value / pi;
	return fRet;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

#include "VECTABLE.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define VECT_TAB_OFFSET_REG     (uint32_t) 0xE000ED08
#define VECTORTABLE_SIZE        (240)
#define VECTORTABLE_ALIGNMENT   (0x100U)
#define VECTORTABLE_ORG_POS		0x0000000
#define FLASH_BASE              ((uint32_t)0x08000000)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t ui32vectorTable_RAM[VECTORTABLE_SIZE] \
                            __attribute__(( aligned (VECTORTABLE_ALIGNMENT) ));
/*******************************************************************************
 * Code
 ******************************************************************************/
/* Moving Vector table to specific location in RAM */
void VEC_TABLE_MoveVECTAB(void) {
    uint32_t ui32i = 0;
    uint32_t *ui32p = (uint32_t*) VECTORTABLE_ORG_POS;
    for (;ui32i < VECTORTABLE_SIZE + 1; ui32i++) {
        ui32vectorTable_RAM[ui32i] = *ui32p;
        ui32p ++;
    }
    /* relocate vector table */
    ui32p = (uint32_t *) VECT_TAB_OFFSET_REG;
    __asm("CPSID   I");
    *ui32p |= (uint32_t) (ui32vectorTable_RAM);
    __asm("CPSIE   I");
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

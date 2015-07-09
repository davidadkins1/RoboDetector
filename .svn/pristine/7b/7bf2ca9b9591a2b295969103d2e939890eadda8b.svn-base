/*********************************************************************
 * FileName:        vectors.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       03/19/05    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "io_cfg.h"

/** V A R I A B L E S ********************************************************/
#pragma udata

/** External  P R O T O T Y P E S ***************************************/
extern void _startup (void);        // See c018i.c in the C18 compiler dir
extern void prvLowInterrupt( void );

/** Local  P R O T O T Y P E S ***************************************/
//void low_isr(void);
//void high_isr(void);

/** V E C T O R  R E M A P P I N G *******************************************/
// The boot strap loader resides at 0 to 0x800
// Setup interrupt and ICD debugger vectors.

#define RM_RESET_VECTOR             0x000800
#define RM_HIGH_INTERRUPT_VECTOR    0x000808
#define RM_LOW_INTERRUPT_VECTOR     0x000818


// These interrupt vectors are for the ICD debugger
//#pragma code _HIGH_INTERRUPT_VECTOR = 0x000008
//void _high_ISR (void)
//{
//    _asm goto RM_HIGH_INTERRUPT_VECTOR _endasm
//}

#pragma code _LOW_INTERRUPT_VECTOR = 0x000018

void _low_ISR (void)
{
    _asm goto RM_LOW_INTERRUPT_VECTOR _endasm
}

// The bootstrap program vectors ISR's here
#pragma code _RESET_INTERRUPT_VECTOR = RM_RESET_VECTOR
void _reset (void)
{
    _asm goto _startup _endasm
}

#pragma code _BOOT_STRAP_HIGH_VECTOR = RM_HIGH_INTERRUPT_VECTOR
void interrupt_at_high_vector(void)
{
//    _asm goto high_isr _endasm
	_asm goto prvLowInterrupt  _endasm
}

#pragma code _BOOT_STRAP_LOW_VECTOR = RM_LOW_INTERRUPT_VECTOR
void interrupt_at_low_vector(void)
{
//    _asm goto low_isr _endasm
	_asm goto prvLowInterrupt  _endasm
}

//#pragma code
/******************************************************************************
 * Function:        void low_isr(void)
 * PreCondition:    None
 * Input:
 * Output:
 * Side Effects:
 * Overview:
 *****************************************************************************/
//#pragma interruptlow low_isr
//void low_isr(void)
//{
//}

/* 12/16/05 - DA - Use interruptlow per Microchip DS51537E */
//#pragma interruptlow high_isr
//void high_isr(void)
//{
//}

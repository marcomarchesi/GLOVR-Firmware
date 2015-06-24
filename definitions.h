/* 
 * File:   definitions.h
 * Author: Emilio Ferrari, Simone Faccini
 *
 */

#ifndef DEFINITIONS_H
#define	DEFINITIONS_H

#ifdef	__cplusplus
extern "C" {
#endif


// *****************************************************************************
// *************************** LIBRARIES INCLUDED ******************************
// *****************************************************************************

// ----- dsPIC33EP512MU810 -----
#include "Compiler.h"
#include <libpic30.h>   // include for delay functions
//#include "define.h"
#include <xc.h>

// ----- Standard libraries -----
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// ----- Hardware specific -----
#include <dsp.h>
#include "uart.h"

// ----- Encoding/Decoding library -----
//#include "G726A.h"
#define AudioBufferSize 10000


// *****************************************************************************
// ***************************** GLOBAL VARIABLES ******************************
// *****************************************************************************


// *****************************************************************************
// ************************* FUNCTION DEFINITIONS ******************************
// *****************************************************************************

extern void Delay_us(unsigned int);
extern void Interrupt_Init(void);
extern void InitializeSystem(void);
extern void Timer_Init(void);
extern void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void);
extern void Timer2_Init(void);
extern void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void);
extern void InitAdc1(void);
extern void __attribute__((interrupt,no_auto_psv)) _AD1Interrupt(void);
extern void InitUART1(void);
extern void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void);
extern void __attribute__ ((interrupt,no_auto_psv)) _U1RXInterrupt(void);
extern void __attribute__((interrupt,no_auto_psv)) _U2TXInterrupt(void);
extern void __attribute__ ((interrupt,no_auto_psv)) _U2RXInterrupt(void);

// *****************************************************************************


#ifdef	__cplusplus
}
#endif

#endif	/* DEFINITIONS_H */


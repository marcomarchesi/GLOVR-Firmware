/* 
 * File:   main.h
 * Author: Andrea Verdecchia
 *
 * Created on 4 giugno 2013, 17.48
 */

#ifndef MAIN_H
#define	MAIN_H

//----- dsPIC33EP512MU810 -----
#include "Compiler.h"
#define FCY 10000000UL  // device operating frequency, Fcy = Fosc/2 = 60MHz
#include <libpic30.h>   // include for delay functions
#include "types.h"

//----- Standard libraries -----
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include <stdint.h>

// Defines
#define SOH 0x01
#define STX 0x02
#define ETX 0x03

#define IMU_DATA_SIZE 14

#define BT_RST LATBbits.LATB15
#define SYS_ON_OFF LATCbits.LATC14
#define FSYNC LATBbits.LATB9

// Emulation EEPROM
#include "DEE Emulation 16-bit.h"

#include "definitions.h"

//MPU9250 libraries
#include "MPU9250.h"

// Madgwick libraries
#include "MadgwickAHRS.h"

void InitializeSystem(void);
void UART_bt_init(void);
void Interrupt_init(void);
void I2C_init(void);

#endif	/* MAIN_H */


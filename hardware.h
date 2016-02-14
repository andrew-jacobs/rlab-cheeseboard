//==============================================================================
//  ____  _          _                                          
// |  _ \| |    __ _| |__                                       
// | |_) | |   / _` | '_ \.                                       
// |  _ <| |__| (_| | |_) |                                     
// |_|_\_\_____\__,_|_.__/          ____                      _ 
//  / ___| |__   ___  ___  ___  ___| __ )  ___   __ _ _ __ __| |
// | |   | '_ \ / _ \/ _ \/ __|/ _ \  _ \ / _ \ / _` | '__/ _` |
// | |___| | | |  __/  __/\__ \  __/ |_) | (_) | (_| | | | (_| |
//  \____|_| |_|\___|\___||___/\___|____/ \___/ \__,_|_|  \__,_|
//                                                              
// A Network Enabled Scrolling LED Sign
//------------------------------------------------------------------------------
// Copyright (C)2016 Andrew Jacobs
// All rights reserved.
//
// This work is made available under the terms of the Creative Commons
// Attribution-NonCommercial-ShareAlike 4.0 International license. Open the
// following URL to see the details.
//
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
//==============================================================================
// Notes:
//
//------------------------------------------------------------------------------

#ifndef HARDWARE_H
#define	HARDWARE_H

#include "xc.h"

#ifdef	__cplusplus
extern "C" {
#endif

//==============================================================================
// Oscillator Configuration
//------------------------------------------------------------------------------

#define OSC             7370000L            // Internal FRC oscillator
#define OSC_PLLDIV      43
#define OSC_PLLPRE      2
#define OSC_PLLPOST     2
    
#define FOSC        (OSC * OSC_PLLDIV / (OSC_PLLPRE * OSC_PLLPOST))          

//==============================================================================
// SPI pin assignments
//------------------------------------------------------------------------------
    
#define SPI_TRIS        TRISB
#define SPI_PORT        PORTB
#define SPI_LAT         LATB
#define SPI_SDI_PIN     9
#define SPI_SDO_PIN     8
#define SPI_SCK_PIN     7
#define SPI_SS_PIN      5

#define SPI_SCK_RP      7
#define SPI_SDO_RP      8
#define SPI_SDI_RP      9

// Ethernet Interrupt pin assignments

#define ETH_TRIS        TRISB
#define ETH_PORT        PORTB
#define ETH_INT_PIN     4

// LED ROW Driver pin assignments

#define ROW_TRIS        TRISB
#define ROW_LAT         LATB
#define ROW0_PIN        14
#define ROW1_PIN        15
#define ROW2_PIN        13
#define ROW3_PIN        11
#define ROW4_PIN        6
#define ROW5_PIN        12
#define ROW6_PIN        10

#define DAT_TRIS        TRISB
#define DAT_LAT         LATB
#define DAT0_PIN        3
#define DAT1_PIN        2

#define CTL_TRIS        TRISA
#define CTL_LAT         LATA
#define GATE_PIN        0
#define SCK0_PIN        2
#define SCK1_PIN        3
#define RCK_PIN         1

#define FCY             (FOSC / 2)
#define USEC            (FCY / 1000000L)
#define NSEC            (FCY / 1000000000L)

//==============================================================================
// Timer Configuration
//------------------------------------------------------------------------------

// Timer 1 is used to measure delays (in milliseconds) during task operations.

#define TMR1_HZ         1000
#define TMR1_PRESCALE   1

#define TMR1_PERIOD     (FOSC / (2 * TMR1_HZ * TMR1_PRESCALE))

#if TMR1_PERIOD & 0xffff0000
# error "TMR1 period is bigger than 16-bits"
#endif

//------------------------------------------------------------------------------

#define FRAME_RATE      32

// Timer 2 is used to trigger the output of pixel data to the diaplay so that
// each row of LEDs is updated 60+ times a second.

#define TMR2_HZ         7 * 4 * FRAME_RATE
#define TMR2_PRESCALE   8

#define TMR2_PERIOD     (FOSC / (2 * TMR2_HZ * TMR2_PRESCALE))

#if TMR2_PERIOD & 0xffff0000
# error "TMR2 period is bigger than 16-bits"
#endif

//------------------------------------------------------------------------------

// Timer 3 is used to trigger display pattern updates from the frame FIFO

#define TMR3_HZ         FRAME_RATE
#define TMR3_PRESCALE   256

#define TMR3_PERIOD     (FOSC / (2 * TMR3_HZ * TMR3_PRESCALE))

#if TMR3_PERIOD & 0xffff0000
# error "TMR3 period is bigger than 16-bits"
#endif

//==============================================================================
// SPI Configuration
//------------------------------------------------------------------------------
        
#define SPI_HZ          8000000
#define SPI_PPRE        1
#define SPI_SPRE        (FCY / (SPI_HZ * SPI_PPRE))

#if (SPI_PPRE == 1) && (SPI_SPRE == 1)
# error "Both SPRE and PPRE cannot be 1:1"
#endif

#if (SPI_SPRE < 1) || (SPI_SPRE > 8)
# error "Invalid Secondary prescale value (1-8)"
#endif

#if   SPI_PPRE == 1
#define SPI_PPRE_BITS   (1 << PPRE1)|(1 << PPRE0)
#elif SPI_PPRE == 4
#define SPI_PPRE_BITS   (1 << PPRE1)|(0 << PPRE0)
#elif SPI_PPRE == 16
#define SPI_PPRE_BITS   (0 << PPRE1)|(1 << PPRE0)
#elif SPI_PPRE == 64
#define SPI_PPRE_BITS   (0 << PPRE1)|(0 << PPRE0)
#else
# error "Invalid Primary scaler value (1,4,16,64)"
#endif

#define SPI_CON1_BITS   ((8 - SPI_SPRE) << 2)|SPI_PPRE_BITS  

//==============================================================================
// Constants
//------------------------------------------------------------------------------

// Calculate the amount of memory used to hold one line of pixels and an entire
// frame.

typedef struct {
    struct {
        unsigned short  pixels [7][5];
    } rows [2];
} Frame;

#define PIXEL_SIZE      (5 * 2)
#define HALF_SIZE       (7  * PIXEL_SIZE)
#define FRAME_SIZE      (14 * PIXEL_SIZE)

#define FIFO_SIZE       (16 * FRAME_SIZE)

#define TEXT_SIZE       8192

#ifdef	__cplusplus
}
#endif

#endif
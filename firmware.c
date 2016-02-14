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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

#include "hardware.h"

#pragma config FNOSC=FRCPLL
#pragma config IESO=OFF
#pragma config FCKSM=CSDCMD
#pragma config OSCIOFNC=ON
#pragma config FWDTEN=OFF
#pragma config ICS=PGD1
#pragma config JTAGEN=OFF

#define BSET(VAR,BIT)   ((VAR) |=  (1 << (BIT)))
#define BCLR(VAR,BIT)   ((VAR) &= ~(1 << (BIT)))

//==============================================================================
// Timer1 Interrupt Handler
//------------------------------------------------------------------------------

volatile unsigned short TICKS = 0;

void __attribute__((__interrupt__,__no_auto_psv__)) _T1Interrupt (void)
{
    ++TICKS;
    
    IFS0bits.T1IF = 0;
}

//==============================================================================
// Timer2 Interrupt Handler
//------------------------------------------------------------------------------

static volatile unsigned short rownum    = 0;

Frame frame;

void __attribute__((__interrupt__,__no_auto_psv__)) _T2Interrupt (void)
{
    unsigned short *pTop = frame.rows[0].pixels[rownum];
    unsigned short *pBot = frame.rows[1].pixels[rownum];
    unsigned short count;
    
    BSET (CTL_LAT, GATE_PIN);
    
    switch (rownum) {
        case 0: BCLR (ROW_LAT, ROW6_PIN);
                BSET (ROW_LAT, ROW0_PIN);
                ++rownum; break;
        case 1: BCLR (ROW_LAT, ROW0_PIN);
                BSET (ROW_LAT, ROW1_PIN);
                ++rownum; break;
        case 2: BCLR (ROW_LAT, ROW1_PIN);
                BSET (ROW_LAT, ROW2_PIN);
                ++rownum; break;
        case 3: BCLR (ROW_LAT, ROW2_PIN);
                BSET (ROW_LAT, ROW3_PIN);
                ++rownum; break;
        case 4: BCLR (ROW_LAT, ROW3_PIN);
                BSET (ROW_LAT, ROW4_PIN);
                ++rownum; break;
        case 5: BCLR (ROW_LAT, ROW4_PIN);
                BSET (ROW_LAT, ROW5_PIN);
                ++rownum; break;
        case 6: BCLR (ROW_LAT, ROW5_PIN);
                BSET (ROW_LAT, ROW6_PIN);
                rownum = 0; break;
    }
    
    for (count = 0; count < 5; ++count) {
        unsigned short mask = 0x8000;
         
        do {
            if (*pTop & mask)
                BSET (DAT_LAT, DAT0_PIN);
            else
                BCLR (DAT_LAT, DAT0_PIN);
            
            if (*pBot & mask)
                BSET (DAT_LAT, DAT1_PIN);
            else
                BCLR (DAT_LAT, DAT1_PIN);

            BSET (CTL_LAT, SCK0_PIN);
            BSET (CTL_LAT, SCK1_PIN);
            asm ("nop");
            asm ("nop");
            asm ("nop");
            BCLR (CTL_LAT, SCK0_PIN);
            BCLR (CTL_LAT, SCK1_PIN);            
        } while (mask >>= 1);
        
        pTop++;
        pBot++;
    }
    
    BSET (CTL_LAT, RCK_PIN);
    asm ("nop");
    asm ("nop");
    asm ("nop");
    BCLR (CTL_LAT, RCK_PIN);
    
    BCLR (CTL_LAT, GATE_PIN);
    
    IFS0bits.T2IF = 0;
}

//==============================================================================
// Timer3 Interrupt Handler
//------------------------------------------------------------------------------




//==============================================================================
// Task Switching
//------------------------------------------------------------------------------

static unsigned int     task = 1;
static jmp_buf          tasks [2];

// Switch to the other task
void yield ()
{
    if (!setjmp (tasks [task])) {
        longjmp (tasks [task ^= 1], 1);
    }
}

//==============================================================================
//
//------------------------------------------------------------------------------

void task1 ()
{
    for (;;) {
        yield ();
        
    }
}

//==============================================================================
//
//------------------------------------------------------------------------------

void task2 ()
{
    for (;;) {
        yield ();
        
    }
}

//==============================================================================
// Power on Reset Initialisation
//------------------------------------------------------------------------------

static void initialise ()
{
    // Turn off unused peripherals
    PMD1 = -1;
    PMD2 = -1;
    PMD3 = -1;
    
    PMD1bits.T1MD = 0;
    PMD1bits.T2MD = 0;
    PMD1bits.T3MD = 0;
    PMD1bits.SPI1MD = 0;
    
    // Configure the oscillator
    CLKDIVbits.PLLPRE  = 0;     // 2
    CLKDIVbits.PLLPOST = 0;     // 2
    PLLFBDbits.PLLDIV  = 41;    // 43

    // Remap I/O pins for SPI
    __builtin_write_OSCCONL(OSCCON & 0xbf);
    RPOR3bits.RP7R = 8;         // SCK
    RPOR4bits.RP8R = 7;         // SDO
    RPINR20bits.SDI1R = SPI_SDI_RP;
    __builtin_write_OSCCONL(OSCCON | 0x40);

    // Configure pins
    AD1PCFGL = -1;
    LATA  = 0;
    TRISA = 0;
    LATB  = (1 << SPI_SS_PIN);
    TRISB = (1 << ETH_INT_PIN)|(1 << SPI_SDI_PIN);
    
    // Configure Timer 1
    T1CON = 0;
    PR1   = TMR1_PERIOD - 1;
    TMR1  = 0;
    T1CONbits.TCKPS = 0;
    T1CONbits.TON = 1;
       
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
 
    // Configure Timer 2
    T2CON = 0;
    PR2   = TMR2_PERIOD - 1;
    TMR2  = 0;
    T2CONbits.TCKPS = 1;
    T2CONbits.TON = 1;
       
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    
    // Configure SPI for ENC28J60
    SPI1CON1bits.PPRE  = 3;
    SPI1CON1bits.SPRE  = 8 - SPI_SPRE;            
    SPI1CON1bits.MSTEN = 1;
    SPI1STATbits.SPIEN = 1;
}

// Reserved a chunk of stack space for task1 the snapshot task2.
void task_init ()
{
    char    space [512];
    memset (space, 0, sizeof (space));

    task2 ();
}

int main (void)
{
    initialise ();
    
    frame.rows[0].pixels[0][0] = frame.rows[1].pixels[0][0] = 0x0101;
    frame.rows[0].pixels[0][1] = frame.rows[1].pixels[0][1] = 0x0101;
    frame.rows[0].pixels[0][2] = frame.rows[1].pixels[0][2] = 0x0101;
    frame.rows[0].pixels[0][3] = frame.rows[1].pixels[0][3] = 0x0101;
    frame.rows[0].pixels[0][4] = frame.rows[1].pixels[0][4] = 0x0101;
    
    frame.rows[0].pixels[1][0] = frame.rows[1].pixels[1][0] = 0x0303;
    frame.rows[0].pixels[1][1] = frame.rows[1].pixels[1][1] = 0x0303;
    frame.rows[0].pixels[1][2] = frame.rows[1].pixels[1][2] = 0x0303;
    frame.rows[0].pixels[1][3] = frame.rows[1].pixels[1][3] = 0x0303;
    frame.rows[0].pixels[1][4] = frame.rows[1].pixels[1][4] = 0x0303;
    
    frame.rows[0].pixels[2][0] = frame.rows[1].pixels[2][0] = 0x0707;
    frame.rows[0].pixels[2][1] = frame.rows[1].pixels[2][1] = 0x0707;
    frame.rows[0].pixels[2][2] = frame.rows[1].pixels[2][2] = 0x0707;
    frame.rows[0].pixels[2][3] = frame.rows[1].pixels[2][3] = 0x0707;
    frame.rows[0].pixels[2][4] = frame.rows[1].pixels[2][4] = 0x0707;
    
    frame.rows[0].pixels[3][0] = frame.rows[1].pixels[3][0] = 0x0f0f;
    frame.rows[0].pixels[3][1] = frame.rows[1].pixels[3][1] = 0x0f0f;
    frame.rows[0].pixels[3][2] = frame.rows[1].pixels[3][2] = 0x0f0f;
    frame.rows[0].pixels[3][3] = frame.rows[1].pixels[3][3] = 0x0f0f;
    frame.rows[0].pixels[3][4] = frame.rows[1].pixels[3][4] = 0x0f0f;
    
    frame.rows[0].pixels[4][0] = frame.rows[1].pixels[4][0] = 0x1f1f;
    frame.rows[0].pixels[4][1] = frame.rows[1].pixels[4][1] = 0x1f1f;
    frame.rows[0].pixels[4][2] = frame.rows[1].pixels[4][2] = 0x1f1f;
    frame.rows[0].pixels[4][3] = frame.rows[1].pixels[4][3] = 0x1f1f;
    frame.rows[0].pixels[4][4] = frame.rows[1].pixels[4][4] = 0x1f1f;
    
    if (!setjmp (tasks [0])) {
        task_init ();
    }
    task1 ();

    return (0);
}
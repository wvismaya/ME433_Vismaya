/* 
 * File:   main.c
 * Author: Vismaya Walawalkar
 *
 * Created on April 18, 2017, 10:16 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include<stdio.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
/*
 * 
 */
#include "ILI9163C.h"
#include "blinky.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF  // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1              /////// slowest wdt?? PS1048576 
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock (8MHz) to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0101      // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


int main() {
   __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0; 
    
    //Initialize the blinky
    init_blinky();
    
    // Initialize the SPI module
    SPI1_init();
   
    //Initialize LCD interface
    LCD_init();
    
    __builtin_enable_interrupts();
    
    // Set background color
    LCD_clearScreen(BLACK);
    
    // Set bar
    LCD_drawBar(128/2, 128/2, 20, 1, RED); // Draw a line
    LCD_drawBar(128/2, 128/2, 20, 0, CYAN); // Draw a line
    
    //Write Hi!
    while(1) {
        delay_core_timer(1000);
        //LATAINV=0b10000;
        toggle_A_pin(4);
        while(PORTBbits.RB4); // Pin B4 is HIGH if pressed.
    }  
}

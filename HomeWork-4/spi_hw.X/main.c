/* 
 * File:   main.c
 * HOMEWORK 4
 * Author: Vismaya Walawalkar
 *
 * Created on April 11, 2017, 9:25 AM
 */
#include<stdio.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
/*
 * 
 */
#include"blinky.h"
#include"spi.h"
// PIC32MX250F128B Configuration Bit Settings

// 'C' source line config statements
// DEVCFG0
#pragma config DEBUG = 0b11 // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_12 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 2017 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module
// #pragma config statements should precede project file includes.


static volatile int Acounter;
static volatile int Bcounter;

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Updater(void){ // interrupt for Timer2
    setVoltage(0,sinWave(Acounter,AFREQ,UPVALS)); //update VoutA with sin function
    setVoltage(1,triWave(Bcounter,BFREQ,UPVALS)); // update VoutB with triangle function
    
    Acounter++;
    Bcounter++;
    
    if(Acounter==((int)((double)(UPVALS/AFREQ)))){
        Acounter=0;
    }
    
    if(Bcounter==((int)((double)(UPVALS/BFREQ)))){
        Bcounter=0;
    }
    
    IFS0bits.T2IF=0; // clear Timer1 interrupt flag
}

int main(){
     __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    dac_init(); // initialize SPI peripheral
    
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4=1; //making B4 into PORT for usrbutton
    
    TRISAbits.TRISA4=0; // making A4 into Out for LED
    
    T2CONbits.TCKPS = 2;  // set Timer2 prescaler
    PR2 = 11999;              // 48MHz/(1kHz * 2*(prescaler))-1
    TMR2 = 0;             // set Timer2 to 0
    IPC2bits.T2IP = 5;   // set Timer2 interrupt priority to 5
    IPC2bits.T2IS = 0;   // set Timer2 sub priority
    IFS0bits.T2IF = 0;   // clear Timer2 interrupt flag
    IEC0bits.T2IE = 1;  // enable Timer2 interrupt
    T2CONbits.ON = 1;  // Turn on  Timer2
    __builtin_enable_interrupts();
    
    LATAbits.LATA4=1;
    
    init_blinky();
    
    while(1) {
        delay_core_timer(1000);
        //LATAINV=0b10000;
        toggle_A_pin(4);
        while(PORTBbits.RB4); // Pin B4 is HIGH if pressed.
    }
   return 0; 
}
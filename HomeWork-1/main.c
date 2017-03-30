// Yay! Part one done till Setting up IDE. Now for blinky!

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

//// DEVCFG0
//#pragma config DEBUG = x // no debugging
//#pragma config JTAGEN = x // no jtag
//#pragma config ICESEL = x // use PGED1 and PGEC1
//#pragma config PWP = 1 // no write protect
//#pragma config BWP = 1 // no boot write protect
#pragma config CP = 1 // no code protect
//
//// DEVCFG1
//#pragma config FNOSC = x // use primary oscillator with pll
//#pragma config FSOSCEN = x // turn off secondary oscillator
//#pragma config IESO = x // no switching clocks
//#pragma config POSCMOD = x // high speed crystal mode
//#pragma config OSCIOFNC = x // disable secondary osc
//#pragma config FPBDIV = x // divide sysclk freq by 1 for peripheral bus clock
//#pragma config FCKSM = x // do not enable clock switch
//#pragma config WDTPS = x // use slowest wdt
//#pragma config WINDIS = x // wdt no window mode
//#pragma config FWDTEN = x // wdt disabled
//#pragma config FWDTWINSZ = x // wdt window at 25%
//
//// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
//#pragma config FPLLIDIV = x // divide input clock to be in range 4-5MHz
//#pragma config FPLLMUL = x // multiply clock after FPLLIDIV
//#pragma config FPLLODIV = x // divide clock after FPLLMUL to get 48MHz
//#pragma config UPLLIDIV = x // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
//#pragma config UPLLEN = x // USB clock on
//
//// DEVCFG3
//#pragma config USERID = 0 // some 16bit userid, doesn't matter what
//#pragma config PMDL1WAY = x // allow multiple reconfigurations
//#pragma config IOL1WAY = x // allow multiple reconfigurations
//#pragma config FUSBIDIO = x // USB pins controlled by USB module
//#pragma config FVBUSONIO = x // USB BUSON controlled by USB module

#define DELAY_TIME 1000

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

    // do your TRIS and LAT commands here
    TRISA = 0x0000;
    TRISB = 0xFFFF;

    __builtin_enable_interrupts();
    int i = 0;

    while(1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
          // remember the core timer runs at half the sysclk
        //LATA = 0x0000;
        while(!PORTBbits.RB7){
            LATAbits.LATA1 = 0xFFFF;
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() <= DELAY_TIME);
            //for(i=0; i<=1000; i++);
            LATAbits.LATA1 = 0x0000;
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() <= DELAY_TIME);
            //for(i=0; i<=1000; i++);   
        }
    }
}

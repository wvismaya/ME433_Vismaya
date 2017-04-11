/* 
 * File:   main.c
 * Author: Vismaya Walawalkar
 *
 * Created on April 11, 2017, 9:25 AM
 */

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h> // including math library
/*
 * 
 */

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
// Use project enums instead of #define for ON and OFF.
#define CS LATBbits.LATB7       // chip select pin, setting to B7
#define UPVALS 1000            // number of points in waveform
#define AFREQ 10           // Frequency for DACA out
#define BFREQ 5           // Frequency for DACB out

static volatile int Acounter;
static volatile int Bcounter;

void delay(void);
unsigned char spi_io(unsigned char o);
void dac_init();
void setVoltage(unsigned char channel, unsigned char voltage);
unsigned char triWave(int count, int freq, int samprate);
unsigned char sinWave(int count, int freq, int samprate);

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
    
    while(1){
         //LATAINV=0b10000;
         setVoltage(0,255);
         setVoltage(1,255);
         delay();
         setVoltage(0,0);
         setVoltage(1,0);
         delay();
    }
   return 0; 
}

void delay(void) {
  int j;
  for (j = 0; j < 1000000; j++) { // number is 1 million
      ;
    while(!PORTBbits.RB4) {
        ;   // Pin B4 is the USER switch, low (FALSE) if pressed.
    }
  }
}
unsigned char spi_io(unsigned char o){
    SPI1BUF = o;
    while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;
}

void dac_init(){
    
  TRISBbits.TRISB7 = 0;
  CS = 1;
  SDI1R = 0b0000; //Setting pin A0 to SS1, although not used
  RPB8R = 0b0011; //Setting pin B8 to SDO1
    
  SPI1CON = 0; // turn off and reset SPI
  SPI1BUF; // Clear rx buffer by reading it
  SPI1BRG = 0x1;// trying max DAC baud rate of 20 MHz end up with 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  SPI1CONbits.MODE16 = 0;  // Mode16 has to be zero for 8 bit.
  SPI1CONbits.MODE32 = 0;  // Mode32 has to be zero for 8 bit.
  SPI1CON2bits.AUDEN = 0;  // Auden has to be zero for 8 bit.

}

void setVoltage(unsigned char channel, unsigned char voltage){
    CS=0; // chips elect needs to be low to initiate write
    unsigned char write;
    if(channel==0){
    write = 0b00110000+(voltage>>4); //first 4 bits are write command to channel A
    spi_io(write); 
    write = 0b00000000+((voltage & 0b00001111)<<4); //moving last 4 voltage bits to first 4 (msb)
    spi_io(write);
    }
    else if(channel==1){
    write = 0b10110000+(voltage>>4);
    spi_io(write); //first 4 bits are write command to channel B
    write = 0b00000000+((voltage & 0b00001111)<<4); //moving last 4 voltage bits to first 4 (msb)
    spi_io(write); 
    }
    CS=1; // chip select should be brought back to high to terminate write
}

unsigned char triWave(int count, int freq, int samprate){
    double out;
    char cout;
    out = 255*(count*((double)freq/samprate));
    cout=(char) out;
    return cout;
}

unsigned char sinWave(int count, int freq, int samprate){
    double out;
    char cout;
    out = 127*sin((count*((double)freq/samprate))*2*3.14)+127;
    cout=(char) out;
    return cout;
}
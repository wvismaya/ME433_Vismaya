#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

//Link to header
#include"blinky.h"

void init_blinky(){
    TRISBbits.TRISB4=1; //making B4 into PORT for usrbutton
    TRISAbits.TRISA4=0; //making A4 into LAT for LED output
    LATAbits.LATA4=1;
}

void set_A_pin(int pinNumber){
    LATA |= (1<<pinNumber);
}

void clear_A_pin(int pinNumber){
    LATA &= ~(1<<pinNumber);
}

void toggle_A_pin(int pinNumber){
    LATAINV |= (1<<pinNumber);
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
#include <stdio.h>
#include "NU32.h"          // constants, functions for startup and 
#include "datainterface.h"
/*  Example of wrapping cos function from math.h with the Python-C-API. */
#define DECIMATE 3              // only send every 4th sample (counting starts at zero)
//#define NSAMPLES 5000           // store 5000 samples

#define PERIOD 1024        // this is PR2 + 1
#define MAXVOLTAGE 3.3     // corresponds to max high voltage output of PIC32

#define MAX_MESSAGE_LENGTH 200

#define VOLTS_PER_COUNT (3.3/1024)
#define CORE_TICK_TIME 25    // nanoseconds between core ticks
#define SAMPLE_TIME 10       // 10 core timer ticks = 250 ns
#define DELAY_TICKS 20000000 // delay 1/2 sec, 20 M core ticks, between messages

volatile int data_buf[10000];// stores the samples
volatile int curr = 0;          // the current index into buffer
unsigned int sample15 = 0;
unsigned int sample_end = 2000;
char buffer[100] = {};

/**********************  ADC FUNCTIONS **********************************************/
unsigned int adc_sample_convert(int pin) { // sample & convert the value on the given 
                                           // adc pin the pin should be configured as an 
                                           // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = pin;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) { 
      ;                                   // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    return ADC1BUF0;                      // read the buffer with the result
}


/**********************  PWM FUNCTIONS **********************************************/
int getUserPulseWidth(void) {
  char msg[100] = {};
  float f = 0.0;

  sprintf(msg, "Enter the desired voltage, from 0 to %3.1f (volts): ", MAXVOLTAGE);
  NU32_WriteUART3(msg);

  NU32_ReadUART3(msg,10);
  sscanf(msg, "%f", &f);
                                  
  if (f > MAXVOLTAGE) {   // clamp the input voltage to the appropriate range
    f = MAXVOLTAGE;
  } else if (f < 0.0) {
    f = 0.0;
  }

  sprintf(msg, "\r\nCreating %5.3f volts.\r\n", f);
  NU32_WriteUART3(msg);
  return PERIOD * (f / MAXVOLTAGE);  // convert volts to counts
}

/**********************  INTERRUPTS FUNCTIONS **********************************************/

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1ISR(void) {  // Timer1 ISR operates at 5 kHz
  static int count = 0;         // counter used for decimation
  static int i = 0;             // the data returned from the isr
  ++i;                          // generate the data (we just increment it for now)
  if(count == DECIMATE) {       // skip some data
    count = 0;
    //if(curr < NSAMPLES) {
      sample15 = adc_sample_convert(15);
      
      if(!PORTDbits.RD7) {
      sprintf(buffer,"%d\r\n",sample_end);  // send the data to the terminal
      NU32_WriteUART3(buffer);
      
      __builtin_disable_interrupts();
      T1CONbits.ON  = 0; 
      IEC0bits.T1IE = 0;
      // sprintf(buffer,"%d\r\n",sample_end);  // send the data to the terminal
      // NU32_WriteUART3(buffer);
      sprintf(buffer,"%d\r\n",sample_end);  // send the data to the terminal
      NU32_WriteUART3(buffer);
      } // Pin D7 is the USER switch, low (FALSE) if pressed.
      else{
        sprintf(buffer,"%d\r\n",sample15);      // send the number of samples that will be sent
        NU32_WriteUART3(buffer);
      }
    }
      //data_buf[curr] = (int) sample15;       // queue a number for sending over the UART
      OC3RS = sample15;
      ++curr;
  // }
  //}
  ++count;
  IFS0bits.T1IF = 0;            // clear interrupt flag
}


int main(void) {
  int NSAMPLES = 0;
  int i = 0;
  
  NU32_Startup();                // cache on, interrupts on, LED/button init, UART init

  __builtin_disable_interrupts();// INT step 2: disable interrupts at CPU
  T1CONbits.TCKPS = 0b01;        // PBCLK prescaler value of 1:8
  PR1 = 9999;                    // The frequency is 80 MHz / (8 * (1999 + 1)) = 5 kHz
  TMR1 = 0;       
  IPC1bits.T1IP = 5;             // interrupt priority 5
  IFS0bits.T1IF = 0;             // clear the interrupt flag
  IEC0bits.T1IE = 1;             // enable the interrupt
  T1CONbits.ON  = 1;             // turn the timer on                                 
  __builtin_enable_interrupts(); // INT step 7: enable interrupts at CPU

  AD1PCFGbits.PCFG14 = 0;                 // AN14 is an adc pin
  AD1PCFGbits.PCFG15 = 0;                 // AN15 is an adc pin
  AD1CON3bits.ADCS = 2;                   // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
                                          //                           2*3*12.5ns = 75ns

  AD1CON1bits.ADON = 1;                   // turn on A/D converter

  PR2 = PERIOD - 1;       // Timer2 is OC3's base, PR2 defines PWM frequency, 78.125 kHz
  TMR2 = 0;               // initialize value of Timer2
  T2CONbits.ON = 1;       // turn Timer2 on, all defaults are fine (1:1 divider, etc.)
  OC3CONbits.OCTSEL = 0;  // use Timer2 for OC3
  OC3CONbits.OCM = 0b110; // PWM mode with fault pin disabled
  OC3CONbits.ON = 1;      // Turn OC3 on

  OC2CONbits.OCTSEL = 0;  // use Timer2 for OC2
  OC2CONbits.OCM = 0b101; // Dual comparee mode; Continuos mode
  OC2CONbits.ON = 1;      // Turn OC2 on

  OC4CONbits.OCTSEL = 0;  // use Timer2 for OC4
  OC4CONbits.OCM = 0b101; // Dual comparee mode; Continuos mode
  OC4CONbits.ON = 1;      // Turn OC4 on

  OC2RS = PERIOD / 2;
  OC4R = PERIOD / 2;

  NU32_ReadUART3(buffer, sizeof(buffer)); // wait for the user to press enter 
  //while(curr !=NSAMPLES) { ; }            // wait for the data to be collected
  while(1){;}

  //NSAMPLES = curr;
  
  /*sprintf(buffer,"%d\r\n",NSAMPLES);      // send the number of samples that will be sent
  NU32_WriteUART3(buffer);

  for(i = 0; i < NSAMPLES; ++i) {
    sprintf(buffer,"%d\r\n",data_buf[i]);  // send the data to the terminal
    NU32_WriteUART3(buffer);
  }*/
  return 0;
}


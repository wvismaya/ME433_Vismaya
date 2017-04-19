/* 
 * File:   blinky.h
 * Author: Vismaya Walawalkar
 *
 * Created on April 11, 2017, 10:57 AM
 */

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

#ifndef BLINKY_H
#define	BLINKY_H

#ifdef	__cplusplus
extern "C" {
#endif
    
void delay_core_timer(int delay_value){
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<delay_value);
}

void delay(void);
//void delay_core_timer(int delay_value);
void init_blinky(); /*Currently for buttons B4 and A4 but will generalize it soon*/
void set_A_pin(int pinNumber);
void clear_A_pin(int pinNumber);
void toggle_A_pin(int pinNumber);

#ifdef	__cplusplus
}
#endif

#endif	/* BLINKY_H */


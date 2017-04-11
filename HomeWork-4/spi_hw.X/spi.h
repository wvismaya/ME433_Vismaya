/* 
 * File:   spi.h
 * Author: Vismaya Walawalkar
 *
 * Created on April 11, 2017, 10:42 AM
 */

#ifndef SPI_H
#define SPI_H

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h> // including math library

#ifdef  __cplusplus
extern "C" {
#endif

// Use project enums instead of #define for ON and OFF.
#define CS LATBbits.LATB7       // chip select pin, setting to B7
#define UPVALS 1000            // number of points in waveform
#define AFREQ 10           // Frequency for DACA out
#define BFREQ 5           // Frequency for DACB out
    
void delay(void);
unsigned char spi_io(unsigned char o);
void dac_init();
void setVoltage(unsigned char channel, unsigned char voltage);
unsigned char triWave(int count, int freq, int samprate);
unsigned char sinWave(int count, int freq, int samprate);




#ifdef  __cplusplus
}
#endif

#endif  /* SPI_H */


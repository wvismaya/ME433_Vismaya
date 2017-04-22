/* 
 * File:   main.c
 * Author: Vismaya Walawalkar
 *
 * Created on April 19, 2017, 1:25 PM
 */

#include <stdlib.h>
#include <stdio.h>
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
/*
 * 
 */
#include "blinky.h"
#include "i2c.h"
#include "ILI9163C.h"

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
    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    //i2c_master_setup();
    //i2c_write(MCP23008,0x05,0b00100000);
    //Initialize the blinky
    init_blinky();
    
    // Initialize the SPI module
    SPI1_init();
   
    //Initialize LCD interface
    LCD_init();
    
    //Init I2C
    i2c_master_setup();
    

    // Set background color
    LCD_clearScreen(BLACK);
    __builtin_enable_interrupts();
    /*Write string of arbitary characters*/
    unsigned short x0 = 28;
    unsigned short y0 = 32;
    int dd,len0;
    unsigned short total_time, ij;
    
    for(dd = 0; dd<1000000; dd++);
    LCD_writechar(45, 20, "IMU");
    LCD_writechar(10, 32, "GYRO");
    LCD_writechar(80, 32, "ACCL");
    
    unsigned char data1;
    unsigned char IMU_data[14];
    
    unsigned short temp_data, gyroX, gyroY, gyroZ, accX, accY, accZ;
            
    data1 = i2c_read(MCP23008, WHOAMI);
    //Test is device responds
    LCD_writeint(1, 1, data1);
    
    //Initialize the IMU
    i2c_write(MCP23008,CTRL1_XL, 0b10000000); ///1000 for 1.66 kHz sample rate, 00 for 2g sensitivity, 00 for 400kHz baud
    i2c_write(MCP23008,CTRL2_G,0b10000000); //1000 for 1.66 kHz, 00 for 245 dps sensitivity, 
    i2c_write(MCP23008,CTRL3_C,0b00000100); //IF_INC bit 1 will enable the ability to read multiple registers
    
    while(1) {
        
        x0 = 28;
        y0 = 32 + 15;
        len0 = 10;
        ij = 0;
        
        i2c_master_multiread(MCP23008,0x20,14,IMU_data);
        
        temp_data = ((IMU_data[1]<<8)|IMU_data[0]) - 65400;
        LCD_writeint(35, 1, temp_data);
        
        gyroX = (float)((signed short)((IMU_data[3]<<8)|IMU_data[2])/320.00);
        LCD_writeint(10, 60, gyroX);
        
        gyroY = (float)((signed short)((IMU_data[5]<<8)|IMU_data[4])/320.00);
        LCD_writeint(10, 80, gyroY);
        
        gyroZ = (float)((signed short)((IMU_data[7]<<8)|IMU_data[6])/320.00);
        LCD_writeint(10, 100, gyroZ);
        
        accX = ((float)((signed short)((IMU_data[9]<<8)|IMU_data[8])/1.00)/32000.00)*9.8;
        LCD_writeint(80, 60, 100*accX);
        
        accY = ((float)((signed short)((IMU_data[11]<<8)|IMU_data[10])/1.00)/32000.00)*9.8;
        LCD_writeint(80, 80, 100*accY);
        
        accZ = ((float)((signed short)((IMU_data[13]<<8)|IMU_data[12])/1.00)/32000.00)*9.8;
        LCD_writeint(80, 100, 100*accZ);
        
        for(dd = 0; dd<1000000; dd++);
    }  
}
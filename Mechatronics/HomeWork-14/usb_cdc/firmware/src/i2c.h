/* 
 * File:   i2c.h
 * Author: Vismaya Walawalkar
 *
 * Created on April 12, 2017, 6:19 PM
 */

#ifndef I2C_H
#define	I2C_H

#define MCP23008 0b01101011
#define WHOAMI 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12

// Header file for i2c_master_noint.c
// helps implement use I2C1 as a master without using interrupts

void i2c_master_setup(void);                // set up I2C 1 as a master, at 100 kHz
void i2c_master_start(void);                // send a START signal
void i2c_master_restart(void);              // send a RESTART signal
void i2c_master_send(unsigned char byte);   // send a byte (either an address or data)
unsigned char i2c_master_recv(void);        // receive a byte of data
void i2c_master_ack(int val);               // send an ACK (0) or NACK (1)
void i2c_master_stop(void);                    // send a stop

void i2c_write(unsigned char ADDRESS, unsigned char REG, unsigned char data);
unsigned char i2c_read(unsigned char ADDRESS,unsigned char REG);

void initI2C(void);
void initExpander(void);
void setExpander(int pin);
void clearExpander(int pinNumber);
unsigned char getExpander(int pinNumber);              // send a stop

void i2c_master_multiread(unsigned char ADDRESS,unsigned char REGISTER,int length,unsigned char *data);
#endif	/* I2C_H */


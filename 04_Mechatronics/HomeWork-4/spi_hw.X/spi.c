// Link to header file
#include"spi.h"

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

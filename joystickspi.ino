#include <stdio.h>
#include <SPI.h>

/*
joystick module written for the MH-ET attiny88 (88 in a QFN on a breakout board)

reads the x and y axes of a joystick with the ADC to two bytes, puts whether or not the button was pushed into a third byte and pushes this all
through SPI when commanded by the received byte for reading, 'R'

output format:
4 bytes:
B=push button (1 bit)
X=x axis reading (10 bits)
Y=y axis reading (10 bits)
P=parity bit (central controller expects even always)
#=unused

PB####XX XXXXXXXX P#####YY YYYYYYYY

identifies itself with the byte 'J' if prompted by the command for identification, 'X'
*/

volatile bool received;
volatile byte dataSPI;
byte output[4];

void setup() {
  pinMode(MISO,OUTPUT);
  pinMode(MOSI,INPUT);
  pinMode(19,INPUT);
  pinMode(20,INPUT);
  pinMode(21,INPUT);
  //enable SPI
  SPCR |= _BV(SPE);
  //set MSTR to 0 to set to slave mode
  SPCR &= ~_BV(MSTR);
  received = false;
  SPI.attachInterrupt();
}

void loop() {
  //spin until data is received
  while(!received);
    //X is a byte requesting for identification
    if(dataSPI=='X'){
      //ID for joystick ("normal" kind; two axes w/ click)
      dataSPI='J';
      received = false;
    }
    //if something was received and it wasn't a request for identification prepare all the input data for sending over
    else if(dataSPI=='R'){
      //read the x axis
      int x = analogRead(19);
      //write the otherwise unused MSB with a flag for whether or not the button in the joystick was pressed
      if(digitalRead(21)){
      x |= 0x40;
      }
      if(x%2){
        x |=0x80;
      }
      output[0] = highByte(x);
      output[1] = lowByte(x);
      //read the y axis
      int y = analogRead(20);
      if(y%2){
        y |=0x80;
      }
      output[2] = highByte(y);
      output[3] = lowByte(y);
      //send 4 bytes of input data (central controller knows to expect 4 bytes because this is identified as a joystick)
    for(int i = 0; i < 4; i++){
      //spin until the last byte was shifted out
      while(!received);
      //load the buffer with a byte to send over
      dataSPI=output[i];
      received = false;
    }
  }
  //go back to sleep now that it's done
  __asm__ ("sleep");
}

ISR (SPI_STC_vect){
  dataSPI = SPDR;
  received = true; 
}
#include <stdio.h>
#include <SPI.h>

/*
joystick module written for the MH-ET attiny88 (88 in a QFN on a breakout board)

reads the x and y axes of a joystick with the ADC to two bytes, puts whether or not the button was pushed into a third byte and pushes this all
through SPI when commanded by the received byte for reading, 'R'

note: would like to put whether or not the joystick was pressed into one of the two axes bits but that would reduce the resolution of one of
the axes by a tiny (literal) bit. might be worth it, I'll look into it

identifies itself with the bit 'J' if prompted by the command for identification, 'X'
*/

volatile bool received;
volatile byte dataSPI;
byte output[3];

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
  while(!received);
    //X is a byte requesting for identification
    if(dataSPI=='X'){
      //ID for joystick ("normal" kind; two axes w/ click)
      dataSPI='J';
    }
    //if something was received and it wasn't a request for identification prepare all the input data for sending over
    else if(dataSPI=='R'){
      //read the x axis
      output[0] = analogRead(19);
      //read the y axis
      output[1] = analogRead(20);
      //read if the button was pressed
      output[2] = digitalRead(21);
      //this might later send a "ready" byte to the central controller but I think the controller can also just wait a couple microseconds

      //send 3 bytes of input data (central controller knows to expect 3 bytes because this is identified as a joystick)
    for(int i = 0; i < 3; i++){
      //wait until a byte's been shifted have been moved
      while(!received){}
      //load the buffer with the right info
      dataSPI=output[i];
    }
  } 
  //go back to sleep now that it's done
  __asm__ ("sleep");
}

ISR (SPI_STC_vect){
  dataSPI = SPDR;
  received = true; 
}
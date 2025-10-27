#include <stdio.h>
#include <SPI.h>

/*
4 button module written for the MH-ET attiny88 (88 in a QFN on a breakout board)

puts 4 buttons into the low 4 bits of a byte and sends it over when prompted with the command for reading, 'R'

output format:
U=up button
D=down button
L=left button
R=right button
#= unusued

####UDLR

identifies itself with the byte 'B' if prompted by the command for identification, 'X'
*/

#define received SPSR & _BV(SPIF)
volatile byte dataSPI;
byte output;

void setup() {
  pinMode(MISO,OUTPUT);
  pinMode(MOSI,INPUT);
  pinMode(19,INPUT);
  pinMode(20,INPUT);
  pinMode(21,INPUT);
  pinMode(22, INPUT);
  //enable SPI
  SPCR |= _BV(SPE);
  //set MSTR to 0 to set to slave mode
  SPCR &= ~_BV(MSTR);
  SPI.attachInterrupt();
}

void loop(){
  //wait for command to finish coming
  while(!received);
    //X is a byte commanding identification, if it comes transmit what kind of part this is
    if(dataSPI=='X'){
      //ID for a 4 button module is 'B'
      dataSPI='B';
    }
    //if R, the the command to transmit input data is sent, do that
    else if(dataSPI=='R'){
    //initialize 8 bits to 0
    output = 0;
    //for the low 4 bits, set to 1 if the corresponding button's been pressed, the high 4 can be ignored
    if(digitalRead(19)){
      output |= _BV(0);
    }
    if(digitalRead(20)){
      output |= _BV(1);
    }
    if(digitalRead(21)){
      output |= _BV(2);
    }
    if(digitalRead(22)){
      output |= _BV(3);
    }
      //send the byte over
    dataSPI=output;
    }
    //go back to sleep when done
    __asm__ ("sleep");
}


ISR (SPI_STC_vect){
  dataSPI = SPDR;
}
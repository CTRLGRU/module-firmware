#include <SPI.h>

/*
4 button module written for the MH-ET attiny88 (88 in a QFN on a breakout board)

puts 4 buttons into the low 4 bits of a byte and sends it over when prompted with the command for reading, 'R'

output format:
U=up button
D=down button
L=left button
R=right button
P=parity bit (central wants even parity)
#= unusued

P###UDLR

identifies itself with the byte 'B' if prompted by the command for identification, 'X'
*/

void setup() {
  //set MISO to output because hardware doesn't do it automatically
  pinMode(MISO,OUTPUT);
  pinMode(MOSI,INPUT);
  pinMode(SCK,INPUT);
  pinMode(SS,INPUT);

  //4 buttons
  pinMode(19,INPUT);
  pinMode(20,INPUT);
  pinMode(21,INPUT);
  pinMode(22, INPUT);

  //give power to SPI
  PRR &= ~_BV(PRSPI);

  //enable SPI
  SPCR |= _BV(SPE) | _BV(SPIE);

  //set MSTR to 0 to set to slave mode
  SPCR &= ~_BV(MSTR);

  //clear SPI data
  SPDR = 0;

  //enable interrupts
  sei();
}

void loop(){
    __asm__ volatile ("sleep");
}

void identify(){
      SPDR='B';
}

void transmitUserInput(){
//initialize 8 bits to 0
  SPDR = 0;
  //for the low 4 bits, set to 1 if the corresponding button's been pressed, the high 4 can be ignored
  if(digitalRead(19)){
    SPDR |= _BV(0);
  }
  if(digitalRead(20)){
    SPDR |= _BV(1);
  }
  if(digitalRead(21)){
    SPDR |= _BV(2);
  }
  if(digitalRead(22)){
    SPDR |= _BV(3);
  }
  //set parity bit
  parity(&SPDR);
}

void parity(char* outputbuffer){
int num1s = 0;
  for(int i = 0; i < 8; i++){
    if(*outputbuffer & _BV(i)){
      num1s++;
    }
  }
  if(num1s % 2){
    *outputbuffer |= _BV(7);
  }
}

ISR(SPI_STC_vect){
    char incoming = SPDR;         // read incoming byte
    switch(incoming){
    case 'X': //X is a byte commanding identification, if it comes transmit what kind of part this is
    identify();
    break;
    case 'R': //if R, transmit the states of the 4 buttons
    transmitUserInput();
    break;
    default:
    SPDR = 250; //for troubleshooting
    }
}
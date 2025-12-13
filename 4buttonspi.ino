#include <SPI.h>

/*
4 button module written for the MH-ET attiny88 (88 in a QFN on a breakout board)

puts 4 buttons into the low 4 bits of a byte and sends it over when prompted with the command for reading, 'R'

output format:
U=up button
D=down button
L=left button
R=right button
P=parity bit (central wants odd parity)
#= unusued

P111UDLR

identifies itself with the byte 'B' if prompted by the command for identification, 'X'
*/

volatile uint8_t data = 0;

void setup() {
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  pinMode(19, INPUT_PULLUP); //PC0-PC3
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  
  PRR &= ~_BV(PRSPI); //Power to SPI
  SPCR |= _BV(SPE) | _BV(SPIE); //Enable SPI logic
  SPCR &= ~_BV(MSTR); //Mode to slave
  SPDR = 0x0F; //I don't like doing all 0s because it looks the same as a dead line and I learned the hard way to not use any vals that could be ASCII 
  sei(); //Enable global interrupts
}

void loop() {
  uint8_t temp = 0;
  
  if(!(PINC & _BV(0))){
    temp |= _BV(0);
  }
  if(!(PINC & _BV(1))){
    temp |= _BV(1);
  }
  if(!(PINC & _BV(2))){
    temp |= _BV(2);
  }
  if(!(PINC & _BV(3))){
    temp |= _BV(3);
  }
  
  temp |= 0x70; // Set bits 6-4 to 111
  parity(&temp);
  data = temp;
  SPDR = data;
  __asm__ volatile ("sleep");
}

void parity(uint8_t* outputbuffer) {
  int num1s = 0;
  for(int i = 0; i < 8; i++) {
    if(*outputbuffer & _BV(i)) num1s++;
  }
  if(!(num1s & 1)) *outputbuffer |= _BV(7);
}

ISR(SPI_STC_vect) {
  uint8_t incoming = SPDR;

  switch(incoming) {
    case 'X': //identify() (not in a separate function call for speed)
      SPDR = 'B'; // Next read will get 'B'
      break;
      
    case 'R': //pollModule() (ditto)
      SPDR = data; // Next read will get button data
      break;
      
    default:
      SPDR = 0x0F; //error byte that I've found DOESN'T look like proper user input like 'X' or 'R' do in binary
      break;
  }
}
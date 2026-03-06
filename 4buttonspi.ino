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
  pinMode(MISO,OUTPUT); //output line to master
  pinMode(MOSI,INPUT); //input line from master (I LOVE FULL DUPLEX!!)
  pinMode(SCK,INPUT); //clock, advances the register whenever the master pulses it
  pinMode(SS,INPUT); //select line (active low)
  //PC0-PC3
  pinMode(19, INPUT_PULLUP); //up
  pinMode(20, INPUT_PULLUP); //down
  pinMode(21, INPUT_PULLUP); //left
  pinMode(22, INPUT_PULLUP); //right
  //just rewire however you want I usually have it wired in the DDR order anyways
  
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
  parity(&temp); //run the parity calculation function
  data = temp; //put the finished stuff in the data buffer where SPI will grab it from
  __asm__ volatile ("sleep"); //go back to sleep when everything's done
}

void parity(uint8_t* outputbuffer) { //calculates and sets parity
  int num1s = 0;
  for(int i = 0; i < 8; i++) {//iterate through the 8 bits
    if(*outputbuffer & _BV(i)){//and add to the 1s count for every set bit
      num1s++;
  }
  if(!(num1s & 1)){ //if there's an even amount of bits (LSB of num1s is 0), set parity bit so it's odd
    *outputbuffer |= _BV(7);
}

void identify(){
  SPDR='B';
}

void inputRead(){
  SPDR = data;
}

ISR(SPI_STC_vect) {
  uint8_t incoming = SPDR;

  switch(incoming) {
    case 'X': //identifies what the function is
      identify(); // Next read will get 'B'
      break;
      
    case 'R': //used to be a separate function call where the user input would be
    //prepared on demand but that's too slow for the speeds the SPI works at so instead
    //that function just runs on loop now but pretend this is basically transmitUserInput();
      inputRead(); // Next read will get button data
      break;
      
    default:
      SPDR = 0x0F; //error byte that I've found DOESN'T look like proper user input like 'X' or 'R' do in binary
      break;
  }
}
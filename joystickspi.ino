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
P=parity bit (central controller expects odd)
#=unused

PB1111XX XXXXXXXX 000000YY YYYYYYYY

identifies itself with the byte 'J' if prompted by the command for identification, 'X'
*/

char volatile output[4];
uint8_t volatile remaining;

void setup() {
  //set MISO to output because hardware doesn't do it automatically
  pinMode(MISO,OUTPUT);
  pinMode(MOSI,INPUT);
  pinMode(SCK,INPUT);
  pinMode(SS,INPUT);
  //2 axes and pushbutton
  pinMode(19,INPUT);
  pinMode(20,INPUT);
  pinMode(21,INPUT);
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

  remaining = 0;
}

void loop(){
    __asm__ volatile ("sleep");
}

void identify(){
  SPDR='J';
}

void prepareInput(){
  int x = analogRead(19);
    //write the otherwise unused MSB with a flag for whether or not the button in the joystick was pressed
    if(digitalRead(21)){
    x |= 0x40;
    }
    output[0] = highByte(x);
    output[0] |= 0x70; //set the bits in the empty space in the first byte for error checking
    output[1] = lowByte(x);
    //read the y axis
    int y = analogRead(20);
    output[2] = highByte(y);
    output[3] = lowByte(y);
  //set parity bit
  parity();
  SPDR=output[0];
  remaining = 3;
}

void transmitUserInput(){
  SPDR = output[4-remaining--];
}

void parity(){
  //calculate and set parity
  int num1s = 0;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 8; j++){
      if(output[i] & _BV(j)){
        num1s++;
      }
    }
  }
  if(!num1s%2){
    output[0] |= _BV(6);
  }
}

ISR(SPI_STC_vect){
    char incoming = SPDR; // read incoming byte
    switch(incoming){
    case 'X': //X is a byte commanding identification, if it comes transmit what kind of part this is
    identify();
    break;
    case 'R': //if R, transmit the states of the 4 buttons
    prepareInput();
    break;
    default:
    if(remaining > 0){
      transmitUserInput();
    } else{
      SPDR=128;//troubleshooting
    }
  }
}
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

uint8_t volatile initialData[4];
uint8_t volatile intermediateData[4];
uint8_t volatile finalData[4];
uint8_t volatile byteToSend;
bool volatile axis; //false = x true = y

void setup() {
  //set MISO to output because hardware doesn't do it automatically
  pinMode(MISO,OUTPUT);
  pinMode(MOSI,INPUT);
  pinMode(SCK,INPUT);
  pinMode(SS,INPUT);
  //2 axes and pushbutton
  pinMode(19,INPUT); //ADC0
  pinMode(20,INPUT); //ADC1
  pinMode(21,INPUT); //PC2

  //SPI setup
  //give power to SPI
  PRR &= ~_BV(PRSPI);

  //enable SPI
  SPCR |= _BV(SPE) | _BV(SPIE);

  //set MSTR to 0 to set to slave mode
  SPCR &= ~_BV(MSTR);

  //set SPI data to obvious debug byte
  SPDR = 0x0F;

  //ADC setup
  //give power to ADC
  PRR &= ~_BV(PRADC);

  //set ADC clock to 1MHz (or specifically 1/16 the main clock but this is made for a 16MHz clocked MCU)
  ADCSRA &= ~(_BV(ADPS0) | _BV(ADPS1));
  ADCSRA |= _BV(ADPS2);
  //disable auto trigger, enable interrupt
  ADCSRA &= ~_BV(ADATE);
  ADCSRA |= _BV(ADIE);

  //set voltage reference to AVcc
  ADMUX |= _BV(REFS0);

  //make sure result is right adjusted to use the whole 10 bit resolution
  ADMUX &= ~_BV(ADLAR);

  //set channel to ADC0
  ADMUX &= 0xF0;

  //enable the ADC and start its first conversion
  ADCSRA |= (_BV(ADEN) | _BV(ADSC));

  //zero all initial globals
  axis = 0;
  byteToSend=0;

  for(int i = 0; i < 4; i++){
  initialData[i] = 0;
  intermediateData[i] = 0;
  finalData[i] = 0;
  }


  //enable interrupts
  sei();


}

void loop(){
  for(int i = 4; i < 4; i++){ //take finished readings from initial buffer
    intermediateData[i] = initialData[i];
  }

  //work on the intermediate buffer which isn't touched by either interrupt
  if(PINC & _BV(2)){
    intermediateData[0] |= _BV(6);
  }

  intermediateData[0] |= 0x3c;

  parity(intermediateData);


  for(int i = 4; i < 4; i++){ //put finished data in the output buffer for SPI to look at
    finalData[i] = intermediateData[i];
  }

    __asm__ volatile ("sleep");
}

void identify(){
  SPDR='J';
}

void parity(uint8_t* outputBuffer){ //calculates and sets parity
  int num1s = 0;
  for(int i = 0; i < 4; i++) {
    for(int j = 0; i < 8; j++){
      if(outputBuffer[i] & _BV(j)) num1s++;
    }
  }
  if(!(num1s & 1)) outputBuffer[0] |= _BV(7);
}

ISR(ADC_VECT){
  if(axis){
    initialData[3] = ADCL; //put Y reading into buffer
    initialData[2] = ADCH;
    axis=0; //flip which axis will be read next time 
    ADMUX |= _BV(0); //switch the channel
    ADCSRA |= _BV(ADSC); //start the ADC again
  }
  else{
    initialData[1] = ADCL; //put X reading into buffer
    initialData[0] = ADCH;
    axis=1; //flip which axis will be read next time 
    ADMUX &= ~_BV(0); //switch the channel
    ADCSRA |= _BV(ADSC); //start the ADC again
  }
}

ISR(SPI_STC_vect){
    char incoming = SPDR; // read incoming byte
    switch(incoming){
    case 'X': //X is a byte commanding identification, if it comes transmit what kind of part this is
    identify(); 
    break;
    case 'R': //if R, transmit the first byte of the output buffer
    SPDR = finalData[byteToSend=0];
    break;
    default:
    if(byteToSend>3){
      SPDR = 0x0F;
    } else{
      SPDR = finalData[++byteToSend];
    }
  }
}
#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>

/*
joystick module written for the MH-ET attiny88 (88 in a QFN on a breakout board)

reads the x and y axes of a joystick at 8 bit resolution and replaces the LSB of the X axis with whether or not the button was pushed, and the LSB of the Y axis with a parity bit

output format:
4 bytes:
B=push button (1 bit)
X=x axis reading (7 bits)
Y=y axis reading (7 bits)
P=parity bit (1 bit, odd expected)

XXXX XXXB YYYY YYYP

identifies itself with the byte 'J' if prompted by the command for identification, 'X'
*/

int8_t volatile initialData[2];
int8_t volatile finalData[2];
uint8_t volatile byteToSend = 0;

volatile bool calibrate = false;
int8_t xOffset = 0;
int8_t yOffset = 0;


void setup() {
  //setup the SPI pins
  pinMode(MISO,OUTPUT); //output line to master
  pinMode(MOSI,INPUT); //input line from master
  pinMode(SCK,INPUT); //clock, advances the register whenever the master pulses it
  pinMode(SS,INPUT); //select line (active low)

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
  //the production model runs at 8MHz so whatever happens happens I guess
  ADCSRA &= ~(_BV(ADPS0) | _BV(ADPS1));
  ADCSRA |= _BV(ADPS2);
  //disable auto trigger, disable interrupt
  ADCSRA &= ~_BV(ADATE);
  ADCSRA &= ~_BV(ADIE);

  //set voltage reference to AVcc
  ADMUX |= _BV(REFS0);

  //make sure result is left adjusted for 8 bit resolution
  ADMUX |= _BV(ADLAR);

  //set channel to ADC0
  ADMUX &= 0xF0;

  //enable the ADC and start its first conversion
  ADCSRA |= (_BV(ADEN) | _BV(ADSC));

  //zero all initial globals
  byteToSend=0;

  for(int i = 0; i < 2; i++){
  initialData[i] = 0;
  finalData[i] = 0;
  }

  //set up sleep
  SMCR |= _BV(SE);
  SMCR &= ~0x06;

  //disable wdt
  wdt_disable();

  //recall offsets
  xOffset = EEPROM.read(0);
  yOffset = EEPROM.read(1);

  //enable interrupts
  sei();

}

void loop(){

  //This MCU wakes up after it's selected by the central MCU to read its prepared data, and once awake it prepares the data for its next transmission then goes back to sleep
  //The final transmission buffer that gets transmitted and the initial buffer that's constructed first are kept separate so if multiple requests come before the buffer is done being prepared it will just
  //send a duplicate of the last buffer, honestly at the speeds this is happening this shouldn't actually affect the user's experience

  if(calibrate){
    recalibrate();
  }

  initialData[0] = setX() - xOffset; //Read the X axis to the initial buffer
  initialData[1] = setY() - yOffset; //Read the Y axis to the initial buffer
  setButton(); //Set the LSB of the X byte for if the pushbutton has been hit or not
  parity(initialData); //run the parity calculation function on the intermediate buffer, write to LSB of Y byte
  transferDataToSPI(); //Write the initial buffer to the final buffer once preparation is done 

    __asm__ volatile ("sleep"); //go back to sleep when everything's done

}

void setButton(){
    if(PINC & _BV(2)){ //indicate if the built in button on the joystick's been pressed
    initialData[0] |= _BV(0);
  } else{
    initialData[0] &= ~_BV(0);
  }
}

void transferDataToSPI(){
  cli();
  for(int i = 0; i < 2; i++){ //put finished data in the output buffer for SPI to look at
    finalData[i] = initialData[i];
  }
  sei();
}


void parity(volatile int8_t* outputBuffer){ //calculates and sets parity
  int num1s = 0;
  for(int i = 0; i < 2; i++) { //for all both bytes in the buffer
    for(int j = 0; j < 8; j++){ //iterate through their 8 bits each
      if(outputBuffer[i] & _BV(j)){ //and add to the 1s count for every bit
        num1s++;
      }
    }
  }
  if(!(num1s & 1)){ //if there's an even amount of bits (LSB of num1s is 0), set parity bit so it's odd
    outputBuffer[1] |= _BV(0);
  }
}

int8_t setX(){
  ADMUX &= ~_BV(0); //switch the channel to ADC0
  ADCSRA |= _BV(ADSC); //start the ADC
  while(ADCSRA & _BV(ADSC)); //spin until the conversion is done
  return ADCH - 128;
}

int8_t setY(){
  ADMUX |= _BV(0); //switch the channel to ADC1
  ADCSRA |= _BV(ADSC); //start the ADC
  while(ADCSRA & _BV(ADSC)); //spin until the conversion is done
  return ADCH - 128;
}

void identify(){
  SPDR='J';
}

void inputRead(){
  SPDR = finalData[byteToSend=0];
}

void recalibrate(){
  int32_t xTotal = 0;
  int32_t yTotal = 0;
  uint8_t samples = 16;

  for(uint8_t i = 0; i < samples; i++){
    xTotal += setX();
    yTotal += setY();
  }
  xOffset = (xTotal/samples);
  yOffset = (yTotal/samples);
  EEPROM.write(0,xOffset);
  EEPROM.write(1,yOffset);
  calibrate = false;
}

ISR(SPI_STC_vect){
    char incoming = SPDR; // read incoming byte
    switch(incoming){
    case 'X': //X is a byte commanding identification, if it comes transmit what kind of part this is
    identify(); 
    break;
    case 'R':
    inputRead(); //Set byteToSend to 0 and load first byte into the SPI buffer
    break;
    case 'C':
    calibrate = true;
    break;
    default:
    if(byteToSend>1){ //if all the data in the buffer's been sent start sending error bytes
      SPDR = 0x0F;
    } else{ //if there's still data to send send the next byte
      SPDR = finalData[++byteToSend];
    }
  }
}
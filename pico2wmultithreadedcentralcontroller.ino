/*
the central controller (pi pico 2w version), grabs input from all the attached modules and sends it to the paired device through bluetooth

uses both cores separately to handle "external" IO and "internal" IO concurrently

external IO involves communication with the desktop app, or whatever device the FMC is paired with:
determining what device the FMC is paired with, if possible,
translating user input from the internal IO into the desired format for the paired device,
+fixing the controller's input data for the paired console if there's no custom setup matching the current hardware config + paired device
++detecting a custom input mapping that fits the current hardware config + paired device
reporting the controller's current configuration to the desktop app
saving custom input mappings and macros from the desktop app to EEPROM
handling user input through the integral (not swappable) inputs
handling wired or wireless communication with the paired device

internal IO involves:
frequently (minimum 250hz, but faster if possible) compiling all the physical user input from swappable modules
looping through the swappable module channels to command them to send their collected data one at a time
detecting changes in module, marking channels as empty and discarding whatever signals come from their lines until an identification signal is answered properly
having methods for parsing input from all the supported modules
placing all acquired data into an expected format for the external IO processor for further processing
a lot of SPI

now you too can have six processors working in parallel to emulate an xbox 360 controller

*/
#include <pico/multicore.h>
#include <SPI.h>


//throwing these in global memory because it seems a lot more convenient to just have these in global w/ mutexes 
char partMap[4];
char intermediateOutputBuffer[32]; //plenty of RAM to go around so have 8 bytes for every module
//at moment of writing I'm not sure what kind of module would need 8 bytes of space but futureproofing doesn't hurt
//modules start at 0, 8, 16, 24
mutex_t internalBufferInUse;

static uint8_t lineToPin[] = {1,1,1,1}; //set these to whatever pins correspond to the SS pins of the top left, top right, bottom right, bottom left modules


void setup(){


  SPI.begin();
  SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));
  

  mutex_init(&internalBufferInUse);
  for(int i = 0; i<4;i++){
    partMap[i]='X';
  }
  delay(100);
  multicore_launch_core1(core1_internal_comms);
}

//core 1 (internal IO)

void core1_internal_comms(){
  while(1){
    mutex_enter_blocking(&internalBufferInUse);
    for(uint8_t i = 0; i < 4; i++){
      pollModule(i,intermediateOutputBuffer);
    }
    mutex_exit(&internalBufferInUse);
  }
}

//core 0 (external IO)
char mapping[2][4]; //[input/output], [lines 1/2/3/4]

void loop(){
  char finalOutputBuffer[32]; //plenty of RAM to go around so have 8 bytes for every module
  //at moment of writing I'm not sure what kind of module would need 8 bytes of space but futureproofing doesn't hurt
  //modules start at 0, 8, 16, 24



}

char getID(uint8_t line){
  digitalWrite(lineToPin[line],HIGH);
  SPI.transfer('X'); //send the command to ID, discard whatever junk was in the slave's buffer before
  char returnedID = SPI.transfer(0); //retrieve the module's ID and send whatever back down the line
  digitalWrite(lineToPin[line],LOW);
  if(returnedID != 'J' && returnedID != 'B'){ //make this more dynamic in the future but for now there are only two possible modules
    returnedID = 'X';
  }
  return returnedID;
}

void pollModule(uint8_t line, char* bufferPointer){
  uint8_t numBytes=0;
  switch(partMap[line]){
    case 'X':
    partMap[line] = getID(line); //if the line is marked as empty then check if that's still the case
    return;
    break;
    case 'J':
    numBytes=4;
    break;
    case 'B':
    numBytes=1;
    break;
  }
  digitalWrite(lineToPin[line],LOW);
  SPI.transfer('R');
  for(uint8_t i = 0; i < numBytes; i++){
    bufferPointer[(line*8)+i] = SPI.transfer(0);
  }
  digitalWrite(lineToPin[line],HIGH);
}

void setPremadeMapping(uint8_t commercialController,char mapArray[2][4]){
  //set to 0 for Xbox layout, set to 1 for a dualshock layout, will add other premade controller layouts later
  switch(commercialController){
    case(0):
    mapArray[1][0] = 'J';
    mapArray[1][1] = 'B';
    mapArray[1][2] = 'J';
    mapArray[1][3] = 'B';
    break;
    case(1):
    mapArray[1][0] = 'J';
    mapArray[1][1] = 'J';
    mapArray[1][2] = 'B';
    mapArray[1][3] = 'B';
    break;
    //can't think of any kind of default behavior this would have
  }
}

void translateBuffer(char IOmap[2][4], char* outputBuffer){
  for(uint8_t i = 0; i < 4; i++){
    if(IOmap[0][i] == 'X' || IOmap[0][i] == IOmap[1][i]){ //if the actual plugged in device is missing/invalid, or if it matches the desired device, it doesn't need to be translated
      break;
    }
  translateModule(IOmap[0][i], IOmap[1][i], outputBuffer, i);
  }
}

void translateModule(char physMod, char mapMod, char* outputBuffer, uint8_t line){
  switch(physMod){
    case('B'):
    switch(mapMod){
          case('J'):
          translateButtonToJoystick(outputBuffer,line);
          break;
    }
    break;
    case('J'):
    switch(mapMod){
          case('B'):
          translateJoystickToButton(outputBuffer,line);
          break;
    break;
    }
  }
}

void translateButtonToJoystick(char* outputBuffer, uint8_t line){
  bool up = outputBuffer[line] & bit(3);
  bool down = outputBuffer[line] & bit(2);
  bool left = outputBuffer[line] & bit(1);
  bool right = outputBuffer[line] & bit(0);

  uint xAxis = 512 + right*512 - left*512;
  uint yAxis = 512 + up*512 - down*512;
  outputBuffer[(8*line)] = highByte(xAxis);
  outputBuffer[(8*line)+1] = lowByte(xAxis);
  outputBuffer[(8*line)+2] = highByte(yAxis);
  outputBuffer[(8*line)+3] = lowByte(yAxis);
}

void translateJoystickToButton(char* outputBuffer, uint8_t line){
  uint xval = ((outputBuffer[(8*line)] & 0x3F) << 8);
  xval |= outputBuffer[(8*line)+1];
  uint yval = ((outputBuffer[(8*line+2)]) << 8);
  yval |= outputBuffer[(8*line)+3];

  outputBuffer[(8*line)] = 0;

  if(xval > 768){
    outputBuffer[(8*line)] |= bit(3);
  }else if(xval < 256){
    outputBuffer[(8*line)] |= bit(2);
  }
  if(yval < 256){
    outputBuffer[(8*line)] |= bit(1);
  }else if(xval > 768){
    outputBuffer[(8*line)] |= bit(0);
  }
}
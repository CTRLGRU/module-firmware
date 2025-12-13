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
#include <Adafruit_TinyUSB.h>

//throwing these in global memory because it seems a lot more convenient to just have these in global w/ mutexes 
char partMap[4];
uint8_t intermediateOutputBuffer[32]; //plenty of RAM to go around so have 8 bytes for every module
//at moment of writing I'm not sure what kind of module would need 8 bytes of space but futureproofing doesn't hurt
//modules start at 0, 8, 16, 24
mutex_t internalBufferInUse;

static uint8_t lineToPin[] = {0,28,20,15}; //set these to whatever pins correspond to the SS pins of the top left, top right, bottom right, bottom left modules

//usb generic HID gamepad setup
uint8_t const desc_hid_report[] = {TUD_HID_REPORT_DESC_GAMEPAD()};
Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

char mapping[2][4]; //[input/output], [lines 1/2/3/4]

void setup(){

  for(int i = 0; i<4; i++){
    pinMode(lineToPin[i],OUTPUT);
  }

  for(int i = 0; i<4; i++){
    digitalWrite(lineToPin[i],HIGH);
  }


  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  
  usb_hid.setPollInterval(4);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  mapping[1][0] = 'J'; //setting a default mapping
  mapping[1][1] = 'B';
  mapping[1][2] = 'J';
  mapping[1][3] = 'B';


  mutex_init(&internalBufferInUse);
  for(int i = 0; i<4;i++){
    partMap[i]='X';
  }
  delay(10);
  multicore_launch_core1(core1_internal_comms);
}

uint8_t volatile flags = 0;
/*
flags:
0|change output map
1|buffer ready for transfer
2|change mode
3|
4|
5|
6|
7|
*/

//core 1 (internal IO)
void core1_internal_comms(){
  unsigned long timeSinceLast = 0;
  while(1){
    if(millis()-timeSinceLast >= 1){
      mutex_enter_blocking(&internalBufferInUse);
      for(uint8_t i = 0; i < 32; i++){
        intermediateOutputBuffer[i] = 0;
      }

      for(uint8_t i = 0; i < 4; i++){
        pollModule(i,intermediateOutputBuffer);
      }

      flags |= bit(1);
      mutex_exit(&internalBufferInUse);
      timeSinceLast = millis();
    }
  }
}

//core 0 (external IO)

#define COMMERCIAL_LAYOUTS 2
uint8_t currentMapping=0;
uint8_t numCustomMappings = 0;
char outputMode = 'G'; //default to generic USB
 
void loop(){

  if(flags & bit(0)){
    if(numCustomMappings == 0){
      numCustomMappings = countMappingsInROM();
    }
    uint8_t totalMappings = COMMERCIAL_LAYOUTS + numCustomMappings;
    currentMapping++;
    if(currentMapping>=totalMappings){
      currentMapping=0;
    }
    if(outputMode=='G'){
      setMapping(0,mapping); //if the mode is set to generic gamepad just default to xbox controller
    }else{
      setMapping(currentMapping,mapping);
    }
    
    flags &= ~bit(0);
  }


  if(flags & bit(1)){ //data in the buffer, grab it and transmit
    uint8_t finalOutputBuffer[32]; //modules start at 0, 8, 16, 24
    mutex_enter_blocking(&internalBufferInUse);
    for(uint8_t i = 0; i < 32; i++){
        finalOutputBuffer[i] = intermediateOutputBuffer[i];
    }
    for(uint8_t i = 0; i < 4; i++){
      mapping[0][i] = partMap[i];
    }
    flags &= ~bit(1);
    mutex_exit(&internalBufferInUse);

    translateBuffer(mapping, finalOutputBuffer);

    transmitOutput(outputMode, finalOutputBuffer,currentMapping);
    
  }

  #ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
  #endif

}

char getID(uint8_t line){
  digitalWrite(lineToPin[line],LOW);
  SPI.transfer('X'); //send the command to ID, discard whatever junk was in the slave's buffer before
  char returnedID = SPI.transfer(0); //retrieve the module's ID and send whatever back down the line
  digitalWrite(lineToPin[line],HIGH);
  if(returnedID != 'J' && returnedID != 'B'){ //make this more dynamic in the future but for now there are only two possible modules
    returnedID = 'X';
  }
  return returnedID;
}

void pollModule(uint8_t line, uint8_t* bufferPointer){
  uint8_t numBytes=0;
  switch(partMap[line]){
    case 'X':
    partMap[line] = getID(line); //if the line is marked as empty then check if that's still the case then get back to it next cycle
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
  delayMicroseconds(2); //give the slave some time to prepare its data buffer, this delay isn't perceptible or anything but it gives the slave 30 clock cycles or so before the SPI starts clocking again
  for(uint8_t i = 0; i < numBytes; i++){
    bufferPointer[(line*8)+i] = SPI.transfer(0);
  }
  digitalWrite(lineToPin[line],HIGH);

  //error checking time
  int num1s = 0;
  bool xcheck = false;
  bool ycheck = false;
  bool bcheck = false;
  switch(partMap[line]){
    case 'J':
      for(int i = 0; i < 4; i++){
        for(int j = 0; j < 8; j++){
          if(bufferPointer[(8*line)+i] & bit(j)){
            num1s++;
          }
        }
      }
      xcheck = (bufferPointer[(8*line)] & 0x3C) == 0x3C; //check if there are any 0s in the x section's padding
      ycheck = !((bufferPointer[(8*line)+2] & 0xFC)); //check if there are any 1s in the y section's padding
      if(!(num1s%2) || !xcheck || !ycheck){
        getID(line);
        for(uint8_t i = 0; i < 4; i++){//if the data's corrupted just wipe the module's state and data, try again next cycle
          bufferPointer[(8*line)+i]=0;
        }
      }
    break;
    case 'B':
        for(int i = 0; i < 8; i++){
          if(bufferPointer[(8*line)] & bit(i)){
            num1s++;
          }
        }
      bcheck = ((bufferPointer[8*line] & 0x70) == 0x70);
      if(!(num1s%2) || !bcheck){
        getID(line);
        for(uint8_t i = 0; i < 4; i++){ 
          bufferPointer[(8*line)+i]=0;
        }
      }
    break;
  }
}

void setMapping(char mapping, char mapArray[2][4]){
  if(mapping < COMMERCIAL_LAYOUTS){
    setPremadeMapping(mapping,mapArray);
  }
  setCustomMapping(mapping,mapArray);
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

void setCustomMapping(uint8_t customController,char mapArray[2][4]){
  char* customMapInROM = getConfigFromROM(customController);
  for(uint8_t i = 0; i < 4; i++){
    mapArray[0][i] = *(customMapInROM+i);
  }
}

void translateBuffer(char IOmap[2][4], uint8_t* outputBuffer){
  for(uint8_t i = 0; i < 4; i++){
    if(IOmap[0][i] == 'X' || IOmap[0][i] == IOmap[1][i]){ //if the actual plugged in device is missing/invalid, or if it matches the desired device, it doesn't need to be translated
      continue;
    }
  translateModule(IOmap[0][i], IOmap[1][i], outputBuffer, i);
  }
}

void translateModule(char physMod, char mapMod, uint8_t* outputBuffer, uint8_t line){
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

void translateButtonToJoystick(uint8_t* outputBuffer, uint8_t line){
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

void translateJoystickToButton(uint8_t* outputBuffer, uint8_t line){
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

char* getConfigFromROM(uint8_t mappingToFind){
  return NULL;
}

void saveConfigToROM(){
  
}

uint8_t countMappingsInROM(){
  return 0;
}

void transmitOutput(char mode,uint8_t* buffer,bool format){
  switch(outputMode){
    case('G'): //at time of writing there are no other output modes so if there's some other mode input here something is wrong
      transmitGenericUSB(buffer, format);
      break;
  }
}

void transmitGenericUSB(uint8_t* buffer, bool format){
  #ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
  #endif

  if (!TinyUSBDevice.mounted()) { //cancel if the controller isn't actually plugged in w/ USB
    return;
  }

  if(format==0){ //xbox layout, top left = joystick, top right = buttons, bottom left = dpad, bottom right = other joystick
    int lx = ((buffer[0] & 0x3F) << 8);
      lx |= buffer[1];
      lx -= 512;
      lx = (lx * 127)/512;
      gp.x = lx;

      int ly = ((buffer[2]) << 8);
      ly |= buffer[3];
      ly -= 512;
      ly = (ly * 127)/512 * -1;
      gp.y = ly;

      int rx = ((buffer[16] & 0x3F) << 8);
      rx |= buffer[17];
      rx -= 512;
      rx = (rx * 127)/512;
      gp.z = rx;

      int ry = ((buffer[18]) << 8);
      ry |= buffer[19];
      ry -= 512;
      ry = (ry * 127)/512;
      gp.rz = ry;

      //triggers //implement later (when I have actual trigger components)
      gp.rx = 0;
      gp.ry = 0;

      uint8_t dpad = buffer[24] & 0x0F;

      bool up    = dpad & 0x08;
      bool down  = dpad & 0x04;
      bool left  = dpad & 0x02;
      bool right = dpad & 0x01;

      if ((up && down) || (left && right)) {
          gp.hat = 0; // center
      }
      else if (up && right) {
          gp.hat = 2;
      }
      else if (down && right) {
          gp.hat = 4;
      }
      else if (down && left) {
          gp.hat = 6;
      }
      else if (up && left) {
          gp.hat = 8;
      }
      else if (up) {
          gp.hat = 1;
      }
      else if (right) {
          gp.hat = 3;
      }
      else if (down) {
          gp.hat = 5;
      }
      else if (left) {
          gp.hat = 7;
      }
      else {
          gp.hat = 0;
      }
      
      gp.buttons = 0;
      switch(buffer[8]){
        case 0x8: //up
          gp.buttons |= GAMEPAD_BUTTON_Y;
        break;

        case 0x9: //up right
          gp.buttons |= GAMEPAD_BUTTON_Y | GAMEPAD_BUTTON_B;
        break;

        case 0x1: //right
          gp.buttons |= GAMEPAD_BUTTON_B;
        break;

        case 0x5: //down right
          gp.buttons |= GAMEPAD_BUTTON_B | GAMEPAD_BUTTON_A;
        break;

        case 0x4: //down
          gp.buttons |= GAMEPAD_BUTTON_A;
        break;

        case 0x6: //down left
          gp.buttons |= GAMEPAD_BUTTON_A | GAMEPAD_BUTTON_X;
        break;

        case 0x2: //left
          gp.buttons |= GAMEPAD_BUTTON_X;
        break;

        case 0xA: //left up
          gp.buttons |= GAMEPAD_BUTTON_X | GAMEPAD_BUTTON_Y;
        break;
      }
  }

  if(format==1){ //playstation layout, clockwise dpad, bottons, joystick and other joystick
    int lx = ((buffer[24] & 0x3F) << 8);
      lx |= buffer[25];
      lx -= 512;
      lx = (lx * 127)/512;
      gp.x = lx;

      int ly = ((buffer[26]) << 8);
      ly |= buffer[27];
      ly -= 512;
      ly = (ly * 127)/512 * -1;
      gp.y = ly;

      int rx = ((buffer[16] & 0x3F) << 8);
      rx |= buffer[17];
      rx -= 512;
      rx = (rx * 127)/512;
      gp.z = rx;

      int ry = ((buffer[17]) << 8);
      ry |= buffer[19];
      ry -= 512;
      ry = (ry * 127)/512;
      gp.rz = ry;

      //triggers
      gp.rx = 0;
      gp.ry = 0;

      uint8_t dpad = buffer[0] & 0xF;

      bool up    = dpad & 0x8;
      bool down  = dpad & 0x4;
      bool left  = dpad & 0x2;
      bool right = dpad & 0x1;

      if ((up && down) || (left && right)) {
          gp.hat = 0; // center
      }
      else if (up && right) {
          gp.hat = 2;
      }
      else if (down && right) {
          gp.hat = 4;
      }
      else if (down && left) {
          gp.hat = 6;
      }
      else if (up && left) {
          gp.hat = 8;
      }
      else if (up) {
          gp.hat = 1;
      }
      else if (right) {
          gp.hat = 3;
      }
      else if (down) {
          gp.hat = 5;
      }
      else if (left) {
          gp.hat = 7;
      }
      else {
          gp.hat = 0;
      }
      
      gp.buttons = 0;
      switch(buffer[8]){
        case 0x8: //up
          gp.buttons |= GAMEPAD_BUTTON_Y;
        break;

        case 0x9: //up right
          gp.buttons |= GAMEPAD_BUTTON_Y | GAMEPAD_BUTTON_B;
        break;

        case 0x1: //right
          gp.buttons |= GAMEPAD_BUTTON_B;
        break;

        case 0x5: //down right
          gp.buttons |= GAMEPAD_BUTTON_B | GAMEPAD_BUTTON_A;
        break;

        case 0x4: //down
          gp.buttons |= GAMEPAD_BUTTON_A;
        break;

        case 0x6: //down left
          gp.buttons |= GAMEPAD_BUTTON_A | GAMEPAD_BUTTON_X;
        break;

        case 0x2: //left
          gp.buttons |= GAMEPAD_BUTTON_X;
        break;

        case 0xA: //left up
          gp.buttons |= GAMEPAD_BUTTON_X | GAMEPAD_BUTTON_Y;
        break;
      }
  }

  usb_hid.sendReport(0, &gp, sizeof(gp));
}
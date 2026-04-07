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
#include <EEPROM.h>

//important constants
#define NUMMODS 6
#define SWAPPABLEMODS 4
#define STATICMODS 2
#define BYTESPERMOD 3
#define NUMMACROS 8
#define MACROTRIGGERSTAGES 4
#define MACROPLAYBACKSTAGES 10
#define COMMERCIAL_LAYOUTS 2
#define CUSTOM_LAYOUTS 3 //this was originally going to be dynamic but with how much space a macro will take up and how many macros there will be in a mapping this will just be capped at 3


//throwing these in global memory because it seems a lot more convenient to just have these in global w/ mutexes
char partMap[4];

uint8_t intermediateOutputBuffer[NUMMODS][BYTESPERMOD];  //6 modules, 3 bytes each
//swappable modules are 0-3, 4-5 are used for integral parts (face buttons, joysticks, bumpers etc)
mutex_t internalBufferInUse;

static uint8_t lineToPin[] = { 0, 1, 2, 3 };  //set these to whatever pins correspond to the SS pins of the top left, top right, bottom right, bottom left modules
static uint8_t integralButtonPins[] = { 4, 5, 6, 7, 8 }; //lbumper rbumper select start central
//later note: the pi pico's pinout has me just putting all GPIO on the left side I guess but also these can be replaced however
static char supportedIDs[] = { 'B', 'J' }; //throw all supported device IDs in here as they come

//usb generic HID gamepad setup
uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_GAMEPAD() };
Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

char mapping[2][SWAPPABLEMODS];  //[input/output], [lines 1/2/3/4]



typedef struct CustomMapping {
  //map of the 4 intended modules for the output mapping
  char components[SWAPPABLEMODS];

  /*
  triggers for the 8 macros, an 8bit number representing each module should cover every possible input
  so 0-3 are swappables, 4&5 are the integral parts
  the MSB of a given byte represents if the input should be exact or not for the trigger [0=not exact 1=exact]
  and there are up to 4 "stages" in a trigger so it takes a chain of specific inputs to trigger a macro
  for example, for a quarter circle [236] input trigger using the bottom left module should go,
  1. triggers[x][0][2] == 0b10000100, set MSB so up+down, or left+down, or right+down aren't valid trigger stages
  2. triggers[x][1][2] == 0b10000101
  3. triggers[x][2][2] == 0b1000001
  4. triggers[x][3][2] == 0b0000000, or "I don't care what happens on this module in this stage"
  and for all other triggers[x][y], set them to 0x00 for don't cares
  */
  uint8_t triggers[NUMMACROS][MACROTRIGGERSTAGES][NUMMODS];

  /*
  every block of 4 bytes is the intended outputs for each module in a step of the macro,
  and there are 10 steps to a macro. MSB of a byte is whether or not it's exact, or if it
  SETS that module to it exactly, or it's just additive. 
  For example, the byte 0b00000000 means make no changes to the module's input for this step,
  while the byte 0b10000000 means WIPE the input from this module for the step.
  Or, 0b00000001 means add a "right" input on that module, in addition to whatever else is actually
  being input on it, while 0b10000001 means cancel real input and just go right.

  For example, a macro that executes a 236 input attack, assuming a joystick in the bottom left
  and buttons in the top right. 

  1. playbacks[x][0][2] == 0b10000100,
  2. playbacks[x][1][2] == 0b10000101
  3. playbacks[x][2][2] == 0b1000001
  3. playbacks[x][3][2] == 0b0000010, additive instead of forcing just a left button press in case your jump button is on the same module or wtv

  */
  uint8_t playbacks[NUMMACROS][MACROPLAYBACKSTAGES][NUMMODS];  //each macro can have 10 steps, each represented by 6 bytes for the 6 modules in it
} CustomMapping;

void setup() {

  for (int i = 0; i < SWAPPABLEMODS; i++) {
    pinMode(lineToPin[i], OUTPUT);
  }

  for (int i = 0; i < SWAPPABLEMODS; i++) {
    digitalWrite(lineToPin[i], HIGH);
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

  mapping[1][0] = 'J';  //setting a default mapping
  mapping[1][1] = 'B';
  mapping[1][2] = 'J';
  mapping[1][3] = 'B';

  EEPROM.begin(2056);

  analogReadResolution(8);

  mutex_init(&internalBufferInUse);
  for (int i = 0; i < SWAPPABLEMODS; i++) {
    partMap[i] = 'X';
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
3|recalibrate
4|
5|
6|
7|
*/

//core 1 (internal IO)
void core1_internal_comms() {
  unsigned long timeSinceLast = 0;
  while (1) {
    if (millis() - timeSinceLast >= 1) {
      mutex_enter_blocking(&internalBufferInUse);
      for (uint8_t i = 0; i < NUMMODS; i++) {
        for (uint8_t j = 0; j < BYTESPERMOD; j++) {
          intermediateOutputBuffer[i][j] = 0;
        }
      }

      for (uint8_t i = 0; i < SWAPPABLEMODS; i++) {
        pollModule(i, intermediateOutputBuffer);
      }

      pollIntegralParts(intermediateOutputBuffer);

      flags |= bit(1);
      mutex_exit(&internalBufferInUse);
      timeSinceLast = millis();
    }
  }
}

//core 0 (external IO)


uint8_t currentMapping = 0;
char outputMode = 'G';  //default to generic USB
uint8_t totalMappings = COMMERCIAL_LAYOUTS + CUSTOM_LAYOUTS;

void loop() {

  #ifdef TINYUSB_NEED_POLLING_TASK  //first and foremost complete USB tasks
    TinyUSBDevice.task();
  #endif

  while (SerialTinyUSB.available()) {
    serialCommandReceived();
  }

  if (flags & bit(0)) {
    currentMapping++;
    if (currentMapping >= totalMappings) {
      currentMapping = 0;
    }
    if (outputMode == 'G') {
      setMapping(0, mapping);  //if the mode is set to generic gamepad just default to xbox controller
    } else {
      setMapping(currentMapping, mapping);
    }

    flags &= ~bit(0);
  }


  if (flags & bit(1)) {               //data in the buffer, grab it and transmit
    uint8_t finalOutputBuffer[NUMMODS][BYTESPERMOD];  //modules are 0-3, 4&5 are integral parts
    mutex_enter_blocking(&internalBufferInUse);
    for (uint8_t i = 0; i < NUMMODS; i++) {
      for (uint8_t j = 0; j < BYTESPERMOD; j++) {
        finalOutputBuffer[i][j] = intermediateOutputBuffer[i][j];
      }
    }
    for (uint8_t i = 0; i < SWAPPABLEMODS; i++) {
      mapping[0][i] = partMap[i];
    }
    flags &= ~bit(1);
    mutex_exit(&internalBufferInUse);

    translateBuffer(mapping, finalOutputBuffer);

    transmitOutput(outputMode, finalOutputBuffer, currentMapping);
  }
}

char getID(uint8_t line) {
  digitalWrite(lineToPin[line], LOW);
  SPI.transfer('X');                  //send the command to ID, discard whatever junk was in the slave's buffer before
  delayMicroseconds(2);
  char returnedID = SPI.transfer(0);  //retrieve the module's ID and send whatever back down the line
  digitalWrite(lineToPin[line], HIGH);
  if (!verifyModuleID(returnedID)) {
    returnedID = 'X';
  }
  return returnedID;
}

bool verifyModuleID(char testedID){
  for(uint8_t i = 0; i < sizeof(supportedIDs); i++){
    if(testedID==supportedIDs[i]){
      return true;
    }
  }
  return false;
}

void pollModule(uint8_t line, uint8_t bufferPointer[][BYTESPERMOD]) {
  uint8_t numBytes = 0;
  switch (partMap[line]) {
    case 'X':
      partMap[line] = getID(line);  //if the line is marked as empty then check if that's still the case then get back to it next cycle
      return;
      break;
    case 'J':
      numBytes = 2;
      break;
    case 'B':
      numBytes = 1;
      break;
  }
  digitalWrite(lineToPin[line], LOW);
  SPI.transfer('R');
  delayMicroseconds(2);  //give the slave some time to prepare its data buffer, this delay isn't perceptible or anything but it gives the slave 30 clock cycles or so before the SPI starts clocking again
  for (uint8_t i = 0; i < numBytes; i++) {
    bufferPointer[line][i] = SPI.transfer(0);
  }
  digitalWrite(lineToPin[line], HIGH);

  //error checking time
  int num1s = 0;
  bool bcheck = false;
  switch (partMap[line]) {
    case 'J':
      for (int i = 0; i < numBytes; i++) {
        for (int j = 0; j < 8; j++) {
          if (bufferPointer[line][i] & bit(j)) {
            num1s++;
          }
        }
      }
      if (!(num1s % 2)) {
        partMap[line] = getID(line);
        for (uint8_t i = 0; i < BYTESPERMOD; i++) {  //if the data's corrupted just wipe the module's state and data, try again next cycle
          bufferPointer[line][i] = 0;
        }
      }
      break;
    case 'B':
      for (int i = 0; i < 8; i++) {
        if (bufferPointer[line][0] & bit(i)) {
          num1s++;
        }
      }
      bcheck = ((bufferPointer[line][0] & 0x70) == 0x70);
      if (!(num1s % 2) || !bcheck) {
        partMap[line] = getID(line);
        for (uint8_t i = 0; i < BYTESPERMOD; i++) {
          bufferPointer[line][i] = 0;
        }
      }
      break;
  }
}

void setMapping(char mapping, char mapArray[2][SWAPPABLEMODS]) {
  if (mapping < COMMERCIAL_LAYOUTS) {
    setPremadeMapping(mapping, mapArray);
  }
  setCustomMapping(mapping, mapArray);
}

void setPremadeMapping(uint8_t commercialController, char mapArray[2][SWAPPABLEMODS]) {
  //set to 0 for Xbox layout, set to 1 for a dualshock layout, will add other premade controller layouts later
  switch (commercialController) {
    case (0):
      mapArray[1][0] = 'J';
      mapArray[1][1] = 'B';
      mapArray[1][2] = 'J';
      mapArray[1][3] = 'B';
      break;
    case (1):
      mapArray[1][0] = 'J';
      mapArray[1][1] = 'J';
      mapArray[1][2] = 'B';
      mapArray[1][3] = 'B';
      break;
      //can't think of any kind of default behavior this would have
  }
}

void setCustomMapping(uint8_t customController, char mapArray[2][SWAPPABLEMODS]) {
  char *customMapInROM = getConfigFromROM(customController);
  for (uint8_t i = 0; i < 4; i++) {
    mapArray[0][i] = *(customMapInROM + i);
  }
}

void translateBuffer(char IOmap[2][SWAPPABLEMODS], uint8_t outputBuffer[][BYTESPERMOD]) {
  for (uint8_t i = 0; i < SWAPPABLEMODS; i++) {
    if (IOmap[0][i] == 'X' || IOmap[0][i] == IOmap[1][i]) {  //if the actual plugged in device is missing/invalid, or if it matches the desired device, it doesn't need to be translated
      continue;
    }
    translateModule(IOmap[0][i], IOmap[1][i], outputBuffer, i);
  }
}

void translateModule(char physMod, char mapMod, uint8_t outputBuffer[][BYTESPERMOD], uint8_t line) {
  switch (physMod) {
    case ('B'):
      switch (mapMod) {
        case ('J'):
          translateButtonToJoystick(outputBuffer, line);
          break;
      }
      break;
    case ('J'):
      switch (mapMod) {
        case ('B'):
          translateJoystickToButton(outputBuffer, line);
          break;
          break;
      }
  }
}

void translateButtonToJoystick(uint8_t outputBuffer[][BYTESPERMOD], uint8_t line) {
  bool up = outputBuffer[line][0] & bit(3);
  bool down = outputBuffer[line][0] & bit(2);
  bool left = outputBuffer[line][0] & bit(1);
  bool right = outputBuffer[line][0] & bit(0);

  if(up&&down&&left&&right){
    outputBuffer[line][0] = 1;
    outputBuffer[line][1]= 0;
    //if all buttons are pushed center the joystick and click the button
    return;
  }

  int8_t xAxis = 0 + right * 127 - left * 127;
  int8_t yAxis = 0 + up * 127 - down * 127;
  outputBuffer[line][0] = xAxis &= ~0x1;
  outputBuffer[line][1] = yAxis &= ~0x1;

}

void translateJoystickToButton(uint8_t outputBuffer[][BYTESPERMOD], uint8_t line) {
  int8_t xval = (outputBuffer[line][0] &= ~0x1);
  int8_t yval = (outputBuffer[line][1] &= ~0x1);

  if (xval > 64) {
    outputBuffer[line][0] |= bit(3);
  } else if (xval < -64) {
    outputBuffer[line][0] |= bit(2);
  }
  if (yval < -64) {
    outputBuffer[line][0] |= bit(1);
  } else if (yval > 64) {
    outputBuffer[line][0] |= bit(0);
  }
}

char *getConfigFromROM(uint8_t mappingToFind) {
  return NULL;
}

void wipeEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }

  CustomMapping empty;
  for (int i = 0; i < SWAPPABLEMODS; i++) {
    empty.components[i] = 'X';
  }

  for (int i = 0; i < NUMMACROS; i++) {
    for (int j = 0; j < MACROTRIGGERSTAGES; j++) {
      for (int k = 0; k < NUMMODS; k++) {
        empty.triggers[i][j][k] = 0xFF;
      }
    }
  }

  for (int i = 0; i < NUMMACROS; i++) {
    for (int j = 0; j < MACROPLAYBACKSTAGES; j++) {
      for (int k = 0; k < NUMMODS; k++) {
        empty.playbacks[i][j][k] = 0x00;
      }
    }
  }

  for (int i = 0; i < CUSTOM_LAYOUTS; i++) {
    EEPROM.put(sizeof(CustomMapping) * i, empty);
  }
  EEPROM.commit();
}


bool saveConfigToROM() {
  SerialTinyUSB.write("SAVING CONFIGS\n");
  CustomMapping tempStructs[CUSTOM_LAYOUTS];
  const int totalBytes = sizeof(tempStructs);
  int received = 0;
  uint8_t *structsPtr = (uint8_t *)tempStructs;

  int startTime = millis();
  while(received<totalBytes){
    
    if(SerialTinyUSB.available()){
      structsPtr[received++]=(uint8_t) SerialTinyUSB.read();
      startTime=millis();

      if(received%256==0){
        tud_task();
        SerialTinyUSB.write("#");
      }
    }
    

    if((millis() - startTime) > 2000){
      SerialTinyUSB.write("TIMEOUT, TRANSFER CANCELED\n");
      return 1;
    }
  }
  for(uint8_t configNum = 0; configNum < CUSTOM_LAYOUTS; configNum++){
    SerialTinyUSB.write('\n');
    SerialTinyUSB.write("SAVING CONFIG ");
    SerialTinyUSB.println(configNum);
    int addr = sizeof(CustomMapping)*configNum;
    EEPROM.put(addr, tempStructs[configNum]);
  }

  EEPROM.commit();

  SerialTinyUSB.write("\nDONE");
  return 0;
}

void transmitOutput(char mode, uint8_t buffer[][BYTESPERMOD], bool format) {
  switch (outputMode) {
    case ('G'):  //at time of writing there are no other output modes so if there's some other mode input here something is wrong
      transmitGenericUSB(buffer, format);
      break;
    case ('R'):
      transmitRawUSBCDC(buffer);
      break;  
  }
}

void transmitRawUSBCDC(uint8_t buffer[][BYTESPERMOD]) {
  uint8_t *flattened = &buffer[0][0];
  SerialTinyUSB.write(flattened, NUMMODS*BYTESPERMOD);
}

void transmitGenericUSB(uint8_t buffer[][BYTESPERMOD], bool format) {

  if (!TinyUSBDevice.mounted()) {  //cancel if the controller isn't actually plugged in w/ USB
    return;
  }

  if (format == 0) {  //xbox layout, top left = joystick, top right = buttons, bottom left = dpad, bottom right = other joystick
    int8_t lx = buffer[0][0];
    gp.x = lx &= ~0x1;

    int ly = buffer[0][1];
    gp.y = ly &= ~0x1;

    int rx = buffer[1][0];
    gp.z = rx &= ~0x1;

    int ry = buffer[1][1];
    gp.rz = ry &= ~0x1;

    //triggers //implement later (when I have actual trigger components)
    gp.rx = buffer[4][1];
    gp.ry = buffer[4][2];

    uint8_t dpad = buffer[3][0] & 0x0F;

    bool up = dpad & 0x08;
    bool down = dpad & 0x04;
    bool left = dpad & 0x02;
    bool right = dpad & 0x01;

    if ((up && down) || (left && right)) {
      gp.hat = 0;  // center
    } else if (up && right) {
      gp.hat = 2;
    } else if (down && right) {
      gp.hat = 4;
    } else if (down && left) {
      gp.hat = 6;
    } else if (up && left) {
      gp.hat = 8;
    } else if (up) {
      gp.hat = 1;
    } else if (right) {
      gp.hat = 3;
    } else if (down) {
      gp.hat = 5;
    } else if (left) {
      gp.hat = 7;
    } else {
      gp.hat = 0;
    }

    gp.buttons = 0;
    switch (buffer[1][0]) {
      case 0x8:  //up
        gp.buttons |= GAMEPAD_BUTTON_Y;
        break;

      case 0x9:  //up right
        gp.buttons |= GAMEPAD_BUTTON_Y | GAMEPAD_BUTTON_B;
        break;

      case 0x1:  //right
        gp.buttons |= GAMEPAD_BUTTON_B;
        break;

      case 0x5:  //down right
        gp.buttons |= GAMEPAD_BUTTON_B | GAMEPAD_BUTTON_A;
        break;

      case 0x4:  //down
        gp.buttons |= GAMEPAD_BUTTON_A;
        break;

      case 0x6:  //down left
        gp.buttons |= GAMEPAD_BUTTON_A | GAMEPAD_BUTTON_X;
        break;

      case 0x2:  //left
        gp.buttons |= GAMEPAD_BUTTON_X;
        break;

      case 0xA:  //left up
        gp.buttons |= GAMEPAD_BUTTON_X | GAMEPAD_BUTTON_Y;
        break;
    }
  }

  if (format == 1) {  //playstation layout, clockwise dpad, bottons, joystick and other joystick
    int lx = buffer[3][0];
    gp.x = lx  &= ~0x1;

    int ly = buffer[3][1];
    gp.y = ly &= ~0x1;

    int rx = buffer[2][0];
    gp.z = rx  &= ~0x1;

    int ry = buffer[2][2];
    gp.rz = ry &= ~0x1;

    //triggers
    gp.rx = buffer[4][1];
    gp.ry = buffer[4][2];

    uint8_t dpad = buffer[0][0] & 0xF;

    bool up = dpad & 0x8;
    bool down = dpad & 0x4;
    bool left = dpad & 0x2;
    bool right = dpad & 0x1;

    if ((up && down) || (left && right)) {
      gp.hat = 0;  // center
    } else if (up && right) {
      gp.hat = 2;
    } else if (down && right) {
      gp.hat = 4;
    } else if (down && left) {
      gp.hat = 6;
    } else if (up && left) {
      gp.hat = 8;
    } else if (up) {
      gp.hat = 1;
    } else if (right) {
      gp.hat = 3;
    } else if (down) {
      gp.hat = 5;
    } else if (left) {
      gp.hat = 7;
    } else {
      gp.hat = 0;
    }

    gp.buttons = 0;
    switch (buffer[1][0]) {
      case 0x8:  //up
        gp.buttons |= GAMEPAD_BUTTON_Y;
        break;

      case 0x9:  //up right
        gp.buttons |= GAMEPAD_BUTTON_Y | GAMEPAD_BUTTON_B;
        break;

      case 0x1:  //right
        gp.buttons |= GAMEPAD_BUTTON_B;
        break;

      case 0x5:  //down right
        gp.buttons |= GAMEPAD_BUTTON_B | GAMEPAD_BUTTON_A;
        break;

      case 0x4:  //down
        gp.buttons |= GAMEPAD_BUTTON_A;
        break;

      case 0x6:  //down left
        gp.buttons |= GAMEPAD_BUTTON_A | GAMEPAD_BUTTON_X;
        break;

      case 0x2:  //left
        gp.buttons |= GAMEPAD_BUTTON_X;
        break;

      case 0xA:  //left up
        gp.buttons |= GAMEPAD_BUTTON_X | GAMEPAD_BUTTON_Y;
        break;
    }
  }

  usb_hid.sendReport(0, &gp, sizeof(gp));
}

void pollIntegralParts(uint8_t buffer[][BYTESPERMOD]) {
  for(uint8_t i = 0; i < sizeof(integralButtonPins); i++){ //poll the integral buttons
    if(digitalRead(integralButtonPins[i])){
      buffer[4][0] |= bit(i);
    }
  }

  buffer[4][1] = analogRead(A0); //left trigger
  buffer[4][2] = analogRead(A1); //right trigger


}

void serialCommandReceived() {
  char buffer[64];
  uint8_t i=0;
  while (SerialTinyUSB.available() && i < sizeof(buffer) - 1) {
    buffer[i++] = SerialTinyUSB.read();
  }

  buffer[i]='\0';

  if (!strcmp(buffer, "TEST")) {
    testRoutines();
  }

  if (!strcmp(buffer, "PING")) {
    SerialTinyUSB.println("PONG");
  }

  if (!strcmp(buffer, "WIPE")) {
    wipeEEPROM();
  }

  if (!strcmp(buffer, "SAVE")) {
    saveConfigToROM();
  }

  if (!strcmp(buffer, "CONFIGS")) {
    reportSavedConfigs();
  }

  if (!strcmp(buffer, "MODULES")) {
    reportInstalledModules();
  }

  if (!strcmp(buffer, "RAW")) {
    outputMode = 'R';
  }

  if (!strcmp(buffer, "SETMODEUSB")) {
    outputMode = 'G';
  }

}

void testRoutines() {
  SerialTinyUSB.write("Test process started \n");

  moduleDetectionTest();
  moduleTranslationTest();
}

void moduleTranslationTest() {
  SerialTinyUSB.write("Test #2: Module translation \n");
  SerialTinyUSB.write("A hard coded input buffer is going to be treated as if it were entered.\n");
  SerialTinyUSB.write("Note: This TEST is currently disabled because it sometimes crashes.\n");
/*
  static char testMapping[2][4];  //manual buffer setting put in a single iteration loop just so I can fold it
  static uint8_t testBuffer[4][8];
  uint8_t expectedResults[4][8];

  uint8_t *flattenedTest = &testBuffer[0][0];
  uint8_t *flattenedExpected = &expectedResults[0][0];

  for (uint8_t dummy = 0; dummy < 1; dummy++) {
    for (uint8_t i = 0; i < 48; i++) {
      flattenedTest[i] = flattenedExpected[i] = 0;
    }

    testMapping[0][0] = 'J';  //up right
    testMapping[0][1] = 'B';  //down left
    testMapping[0][2] = 'J';  //left up
    testMapping[0][3] = 'B';  //right down

    testMapping[1][0] = 'B';
    testMapping[1][1] = 'J';
    testMapping[1][2] = 'B';
    testMapping[1][3] = 'J';

    testBuffer[0][0] = 0x3F;
    testBuffer[0][1] = 0xFF;
    testBuffer[0][2] = 0x03;
    testBuffer[0][3] = 0xFF;

    testBuffer[1][0] = 0x76;

    testBuffer[2][0] = 0x3C;
    testBuffer[2][1] = 0x00;
    testBuffer[2][2] = 0x03;
    testBuffer[2][3] = 0xFF;

    testBuffer[3][0] = 0x75;

    expectedResults[0][0] = 0x79;

    expectedResults[1][0] = 0x3c;
    expectedResults[1][1] = 0x00;
    expectedResults[1][2] = 0x00;
    expectedResults[1][3] = 0x00;

    expectedResults[2][0] = 0x75;

    expectedResults[3][0] = 0x3F;
    expectedResults[3][1] = 0xFF;
    expectedResults[3][2] = 0x00;
    expectedResults[3][3] = 0x00;
  }

  //print untreated test buffer

  SerialTinyUSB.write(flattenedTest, 48);

  SerialTinyUSB.write("\nCompare with: \n");

  //print known correct results

  SerialTinyUSB.write(flattenedExpected, 48);

  uint8_t errors = 0;
  translateBuffer(testMapping, testBuffer);

  SerialTinyUSB.write(flattenedTest, 48);
  if (memcmp(flattenedTest, flattenedExpected,48)){
    SerialTinyUSB.write("\n FAIL");
  } else {
    SerialTinyUSB.write("\n PASS");
  }
  */

}

void reportSavedConfigs() {
  CustomMapping tempStructs[3];
  for(uint8_t configNum = 0; configNum < 3; configNum++){
    int addr = configNum * sizeof(CustomMapping);
    EEPROM.get(addr, tempStructs[configNum]);
    uint8_t *flattened = (uint8_t *) &tempStructs[configNum];
    for(int byteInConfig = 0; byteInConfig < sizeof(CustomMapping); byteInConfig++){
      SerialTinyUSB.write(flattened[byteInConfig]);
    }
  }
}

void reportInstalledModules() {
  char installed[4];
  for (uint8_t i = 0; i < 4; i++) {
    installed[i] = mapping[0][i];
  }
  SerialTinyUSB.write(installed, 4);
}

void clearSerial(Stream* serial){
  while(serial->available()>0){
    serial->read();
  }
}

void moduleDetectionTest() {
  SerialTinyUSB.write("Test #1: Module detection \n");
  SerialTinyUSB.write("Currently detected modules are: \n");
  for (uint8_t i = 0; i < 4; i++) {
    SerialTinyUSB.write(getID(i));
    SerialTinyUSB.write('\n');
  }
  SerialTinyUSB.write("Please verify visually if that's right\n");
}

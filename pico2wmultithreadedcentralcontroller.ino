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

void setup(){
  
  multicore_launch_core1(core1_internal_comms);
}

//core 1 (internal IO)
void core1_internal_comms(){

}

//core 0 (external IO)
void loop(){

}

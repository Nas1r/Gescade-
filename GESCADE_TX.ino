#include <SPI.h> 
#include "RF24.h"

// Functionality for RF24 Radio transmission from Sensors
// to the Arduino connected to the PC as gaming interface

RF24 myRadio (9, 10);

void radio_setup(){
    myRadio.begin();  
    myRadio.setPALevel(RF24_PA_MAX);
    myRadio.setDataRate( RF24_1MBPS ) ; 
    myRadio.openWritingPipe( addresses[0]);
}

void radio_send(const char& data){
  myRadio.write(&data, sizeof(data));
}

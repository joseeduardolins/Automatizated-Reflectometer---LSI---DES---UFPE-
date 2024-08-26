#include <Arduino.h>
#include "RA.h"
#include "BluetoothSerial.h"

RA RAlib; //instancia da classe RA


void setup() {
RAlib.Begin();
}




void processFunction(String functionName, String argument);

void loop() {
RAlib.EsperaMensagem();
delay(10);
}



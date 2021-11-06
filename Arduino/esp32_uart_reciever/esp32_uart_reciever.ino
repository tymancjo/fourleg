/*  This file is a sketch to test upgraded idea of serial commands recieve
 *  for the 4 leg robot Aron mk2. 
 *  it's based on the idea found on the internet:
 *  SerialCommand - A Wiring/Arduino library, Copyright (C) 2012 Stefan Rado
 * 
 *  2021 - TymancjO
 */

#include "SerialCommand.h"

SerialCommand sCmd;     // The demo SerialCommand object

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Setup callbacks for SerialCommand command
  sCmd.addCommand("c",  processCommand2);  // Converts two arguments to integers and echos them back
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
  
  
  delay(500);
  Serial.println("Ready");
}


void loop() {
  // put your main code here, to run repeatedly:
  sCmd.readSerial();
  
} // loop



// Functions

void processCommand2() {
  // protype function to recieve many arguments (up to 20 here)
  
  int aNumber;
  char *arg;
  int arg_num = 0;
  
  for (int a=0; a<20; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      Serial.print("argument ");
      Serial.print(a);
      Serial.print(" = ");
      Serial.println(aNumber);
    }
    else {
      // we escape as no argument was found. 
      break;
    }
  }
  
Serial.print("Recieved ");
Serial.print(arg_num);
Serial.println(" arguments");
  
} // processCommands2


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("NOK");
}

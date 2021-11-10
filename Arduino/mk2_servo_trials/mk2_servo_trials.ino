/*************************************************** 
  This is based on the example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive servos using  the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "SerialCommand.h"
SerialCommand sCmd;     // The SerialCommand object

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm  = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x60); 


// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
//// have!

#define SERVOMIN 170  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 495  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
const uint8_t servos = 16;


float servo_target[servos] = {  90,90,90,90,  90,90,90,90,  110,70,110,70,  90,90,90,90 };
float servo_home[servos] = {    90,90,90,90,  90,90,90,90,  110,70,110,70,  90,90,90,90 };
float servo_current[servos];
bool    servo_inverse[servos] = { 0,0,1,1,  0,0,1,1,  0,0,1,1,  0,0,0,0 };
float   servo_ramp[servos];

bool move_done = false;
bool in_loop = false;
int waiting = 0;

unsigned long now = millis();
unsigned long last = millis();
unsigned long last_step = millis();

#define LEDPIN 2
#define POWEROUT 12

bool power = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Aron mk2 Servo test!");
  pinMode(LEDPIN, OUTPUT);
  pinMode(POWEROUT, OUTPUT);

  digitalWrite(POWEROUT, power);

  // Setup callbacks for SerialCommand command
  sCmd.addCommand("c",  processCommand);      // Converts two arguments to integers and echos them back
  sCmd.addCommand("test", runTest);           // run the simple test sequence - just moves all servos in loop.
  sCmd.addCommand("h", makeHoming);           // ressetting the servos to initial position
  sCmd.addCommand("ramp", setRamp);           // set up the ramp value input as 990 gives 0.99 ramp etc.
  sCmd.addCommand("s", setServo);             // set a single servo command
  sCmd.addCommand("d", setServoDelta);        // set a single servo by delta angle command
  sCmd.addCommand("ad", setAllServosDelta);   // move all by individual deltas
  sCmd.addCommand("adh", setAllServosDeltaHome);  // move all by individual deltas from home position
  sCmd.addCommand("circ", makeCircle);        // moving front back left right
  sCmd.addCommand("show", showAngles);        // displaying back the current settings
  sCmd.addCommand("swing", makeSwing);        // making a swing to the side
  sCmd.addCommand("twist", makeTwist);        // making a twist to the side
  sCmd.addCommand("power", togglePower);      // toggle the power out
  
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
  

  // preparing the inital positions.
  for (int k = 0; k < servos; k++) {
    //servo_target[k] = 90;
    servo_current[k] = 80;
    servo_ramp[k]= 0.95;
  }

  


  pwm.begin();
  pwm2.begin();

  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  Serial.print("Starting up");
  for (int k=0; k < 21; k++){
    digitalWrite(LEDPIN, HIGH);
    Serial.print(".");
    delay(400 - k*20);
    digitalWrite(LEDPIN, LOW);
    delay(400 - k*20);
  }
  Serial.println();
  digitalWrite(LEDPIN, HIGH);
}

int test_step = 0;
const int test_steps = 3;
int test_pos[test_steps] = {90, 70, 110};
//int test_pos[test_steps] = {90, 90, 90};

void loop() {
// marking time
now = millis();
// making the read from serial
sCmd.readSerial();

  
// testing servo position
if (move_done && in_loop) {
  
  Serial.println("Move is done...");
  Serial.print("Test step ");
  Serial.print(test_step);
  Serial.print(" set angle ");
  Serial.println(test_pos[test_step]);
  
  for (int s=0; s < servos; s++){
    servo_target[s] = test_pos[test_step];
  }
  test_step++;
  
  if (test_step == test_steps){
    test_step = 0;
  }
}



  
// making the servos moves
if (now > last + 2){
  last = now;  
  move_done = true;
  for (int s=0; s < servos; s++){

    // checking if need to move
    if (abs(servo_target[s] - servo_current[s]) > 1){
    
      move_done = false;
      servo_current[s] = servo_ramp[s] * servo_current[s] + (1 - servo_ramp[s]) * servo_target[s];
      set_servo(s, servo_current[s]);

    }
    else {
      servo_current[s] = servo_target[s];
    }
  
  }
}


} // loop


// Functions

void set_servo(int ser, float angle){
  // function to simplify setting the servo positions
  // ser - number of servo
  // angle - angle to set in degrees.
  // sanity check
  if (-1 < angle && angle < 181 && -1 < ser && ser < servos){
      // making the left/right servo detection 
      if (servo_inverse[ser]) angle = 180 - angle;
      
      // getting the pusle lenght
      int pulselen = map(angle,0,180,SERVOMIN,SERVOMAX);
      
      // sending the command to the propper PCA unit
      if (ser < 8){
          pwm2.setPWM(ser, 0, pulselen);
        }
      else
        {
          pwm.setPWM(ser-8, 0, pulselen);
        }  
  }
}


void togglePower(){
  // toggle power for the servos
  power = !power;
  digitalWrite(POWEROUT, power);
}

void makeHoming(){
  // this one stops tle loping and set initial positinos
  in_loop = false;
  Serial.print("Homming ");
  for (int s=0; s < servos; s++){
      servo_target[s] = servo_home[s];
      Serial.print(".");
    }
   Serial.println();
  }

void runTest(){
  // this one toggle the looping of the test set.
  in_loop = !in_loop;
  if (in_loop){
    Serial.println("Running loop...");
    }
  else {
    Serial.println("Stopping loop!");
  }
}


void setRamp() {
  // protype function to recieve ramp and set it for all servos
  
  float aNumber;
  char *arg;
  
  for (int a=0; a<1; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atof(arg);    // Converts a char string to float
      float ramp = aNumber/1000;
      Serial.print("ramp ");
      Serial.print(" = ");
      Serial.println(ramp);

      for( int s=0; s < servos; s++) servo_ramp[s]= ramp;
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK no ramp argument");
      break;
    }
  }
} // setRamp

void setServo() {
  // protype function to recieve many arguments (up to 20 here)
  
  int theValue[2];
  int arg_num = 0;
  char *arg;

  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      theValue[a] = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      Serial.print("argument ");
      Serial.print(a);
      Serial.print(" = ");
      Serial.println(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK  2 args expected");
      break;
    }
  }
  
  if (arg_num == 2) {
    if (-1 < theValue[0] && theValue[0] < servos) {
      if (-1 < theValue[1] && theValue[1] < 181){
        servo_target[theValue[0]] = theValue[1];
      }
    }
  }
} // setServo



void setServoDelta() {
  // protype function to recieve many arguments (up to 20 here)
  
  int theValue[2];
  int arg_num = 0;
  char *arg;

  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      theValue[a] = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      Serial.print("argument ");
      Serial.print(a);
      Serial.print(" = ");
      Serial.println(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK  2 args expected");
      break;
    }
  }
  
  if (arg_num == 2) {
    if (-1 < theValue[0] && theValue[0] < servos) {
      int delta = servo_target[theValue[0]] + theValue[1];
      
      if (-1 < delta && delta < 181){
        servo_target[theValue[0]] = delta;
      }
    }
    else if (theValue[0] == -1){
      // we move all sevos by this delta
      for (int s=0; s < servos; s++){
        int delta = servo_target[s] + theValue[1];
        
        if (-1 < delta && delta < 181){
           servo_target[s] = delta;
          }  
        }
      }
  }
} // setServoDelta



void setAllServosDeltaHome() {
  // protype function to recieve many arguments (up to 20 here)
  
  int theValue[16];
  int arg_num = 0;
  char *arg;

  
  for (int a=0; a<16; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      theValue[a] = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      Serial.print("argument ");
      Serial.print(a);
      Serial.print(" = ");
      Serial.println(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK  16 args expected");
      break;
    }
  }
  
  if (arg_num == 16) {
    // we move all sevos by this delta
      for (int s=0; s < servos; s++){
        
        int alfa = servo_home[s] + theValue[s];
        
        if (-1 < alfa && alfa < 181){
           servo_target[s] = alfa;
          }  
        }
      
  }
} // setAllServosDeltaHome



void makeCircle() {
  // protype function to recieve many arguments (up to 20 here)
  
  int theValue[2];
  int arg_num = 0;
  char *arg;

  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      theValue[a] = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      Serial.print("argument ");
      Serial.print(a);
      Serial.print(" = ");
      Serial.println(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK  2 args expected");
      break;
    }
  }
  
  if (arg_num == 2) {
    // we move all sevos by this delta
      for (int s=0; s < servos; s++){
        
        int alfa = servo_home[s] + theValue[0];
        if (s > 7) alfa = servo_home[s] + theValue[1];
        
        if (-1 < alfa && alfa < 181){
           servo_target[s] = alfa;
          }  
        }
      
  }
} // makeCircle




void setAllServosDelta() {
  // protype function to recieve many arguments (up to 20 here)
  
  int theValue[16];
  int arg_num = 0;
  char *arg;

  
  for (int a=0; a<16; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      theValue[a] = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      Serial.print("argument ");
      Serial.print(a);
      Serial.print(" = ");
      Serial.println(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK  16 args expected");
      break;
    }
  }
  
  if (arg_num == 16) {
    // we move all sevos by this delta
      for (int s=0; s < servos; s++){
        
        int alfa = servo_target[s] + theValue[s];
        
        if (-1 < alfa && alfa < 181){
           servo_target[s] = alfa;
          }  
        }
      
  }
} // setAllServosDelta



void showAngles(){
  // serving the curretn servos angles. 
  Serial.print("current: ");
  for (int s=0; s < servos; s++) {
    Serial.print((int)servo_target[s]);
    if (s < servos-1) Serial.print(",");
  }
  Serial.println();
} // show angles


void makeSwing(){
  // this fuction makes a swing of given angle
  int aNumber;
  char *arg;
  
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      Serial.print("Swing by ");
      Serial.print(aNumber);
      Serial.println(" degrees"); 

      aNumber = constrain(aNumber, -70,70);
      for (int s=8; s < 12; s++) servo_target[s] = servo_home[s] + aNumber;
      
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK");
    }
  
} // makeSwing


void makeTwist(){
  // this fuction makes a swing of given angle
  int aNumber;
  char *arg;
  
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      Serial.print("Swing by ");
      Serial.print(aNumber);
      Serial.println(" degrees"); 

      aNumber = constrain(aNumber, -70,70);
      servo_target[8] = servo_home[8] + aNumber;
      servo_target[9] = servo_home[9] + aNumber;
      servo_target[10] = servo_home[10] - aNumber;
      servo_target[11] = servo_home[11] - aNumber;
      
    }
    else {
      // we escape as no argument was found. 
      Serial.println("NOK");
    }
  
} // makeTwist


void processCommand() {
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
  
} // processCommands



// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("NOK");
}

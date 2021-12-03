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

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// #include <ArduinoJson.h>

// to be able use filesystem
#include "SPIFFS.h"

//#include "html.h"
#include "secret.h"

// Replace with your network credentials
const char* ssid = SSIDs;
const char* password = PASSs;

// ********** Stuff for webservice *********

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


void notifyClients() {
  
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
      command((char*)data);
  }
}



void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var){
  Serial.println(var);
  return "OK";
}

// ******************

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
uint8_t servo_delay = 20;

// float servo_zero[servos]    = {   90,90,90,90,  90,90,90,90,  90,90,90,90,  90,90,90,90 };
// float servo_target[servos]  = {   90,90,90,90,  90,90,90,90,  90,90,90,90,  90,90,90,90 };
// float servo_home[servos]    = {   90,90,90,90,  90,90,90,90,  90,90,90,90,  90,90,90,90 };

float servo_zero[servos]    = {   90,90,90,90,  90,90,90,90,  110,70,110,70,  90,90,90,90 };
float servo_target[servos]  = {   90,90,90,90,  90,90,90,90,  110,70,110,70,  90,90,90,90 };
float servo_home[servos]    = {   90,90,90,90,  90,90,90,90,  110,70,110,70,  90,90,90,90 };
bool  servo_inverse[servos] = {   0,0,1,1,      0,0,1,1,      0,0,1,1,        0,0,0,0 };

// for making the incremental model
float servo_up[servos]      = {   0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
float servo_fb[servos]      = {   0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
float servo_lr[servos]      = {   0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
float servo_legs[servos]    = {   0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

float servo_current[servos];
float servo_ramp[servos];
float servo_step[servos]    = {   2,2,2,2,  2,2,2,2,  2,2,2,2,  2,2,2,2 };

bool move_done = false;
bool in_loop = false;
bool in_linear = false;
int waiting = 0;

unsigned long now = millis();
unsigned long last = millis();
unsigned long last_step = millis();

#define LEDPIN 2
#define POWEROUT 12

#define M1A 33
#define M1B 25
#define M2A 26
#define M2B 27

//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x)    Serial.print (x)
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
#endif

bool power = false;

void setup() {
//  Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("Aron mk2 Firmware System");
  pinMode(LEDPIN, OUTPUT);
  pinMode(POWEROUT, OUTPUT);

  // Preparing the PWM channels for the motors
  ledcAttachPin(M1A, 0); // assign a led pins to a channel
  ledcAttachPin(M1B, 1); // assign a led pins to a channel
  ledcAttachPin(M2A, 2); // assign a led pins to a channel
  ledcAttachPin(M2B, 3); // assign a led pins to a channel

  ledcSetup(0, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(1, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(3, 4000, 8); // 12 kHz PWM, 8-bit resolution

  digitalWrite(POWEROUT, power);

  // Setup callbacks for SerialCommand command
  
  sCmd.addCommand("test", runTest);           // run the simple test sequence - just moves all servos in loop.
  sCmd.addCommand("power", togglePower);      // toggle the power out
  sCmd.addCommand("show", showAngles);        // displaying back the current angles of all servos
  
  sCmd.addCommand("h", makeHoming);           // ressetting the servos to home position
  sCmd.addCommand("reset", makeZero);         // ressetting the servos home to initial position - but no move make
  sCmd.addCommand("ramp", setRamp);           // set up the ramp value input as 990 gives 0.99 ramp etc.
  
  sCmd.addCommand("circ", makeCircle);        // moving front back left right by 2 arguments 
  sCmd.addCommand("move", makeMove);          // triggering to makte move to current incremental set
  sCmd.addCommand("up", makeUp);              // adding the value to the homing of the up/dn servos
  sCmd.addCommand("leg", moveLeg);            // set data for single leg incremental model move
  sCmd.addCommand("drv", driveWheels);        // move the wheels by 2 params
  sCmd.addCommand("s", allStop);              // full stop for wheels
  sCmd.addCommand("twist", makeTwist);        // twist - to make a turning - experimental  
  sCmd.addCommand("walk", makeWalk);          // the sudo walk command
  sCmd.addCommand("w", makeWalk);             // the sudo walk command
  sCmd.addCommand("look", lookAround);        // looking around in LR TD

  sCmd.addCommand("fake", fakeData);          // to test the fake serial data
  sCmd.addCommand("mode", toogleMmode);       // toggle the interpoloation mode
  sCmd.addCommand("step", setStepSize);       // to set the single step size in degrees
  sCmd.addCommand("delay", setDelay);         // to set the each loop servos delay
  
  // sCmd.addCommand("s", setServo);             // set a single servo command
  // sCmd.addCommand("d", setServoDelta);        // set a single servo by delta angle command
  // sCmd.addCommand("ad", setAllServosDelta);   // move all by individual deltas
  // sCmd.addCommand("adh", setAllServosDeltaHome);  // move all by individual deltas from home position
  
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


  // checking the file system
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  
    // Preparing static IP adress 
    IPAddress local_ip0(192,168,0,123);
    IPAddress gateway0(192,168,0,1);
    IPAddress subnet0(255,255,255,0);
    

   // Connect to Wi-Fi
      if (!WiFi.config(local_ip0, gateway0, subnet0)) {
          Serial.println("STA Failed to configure");
          }
          
      WiFi.begin(ssid, password);
      int wifitrys = 0;
      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
        wifitrys++;
    
        if (wifitrys > 7) break;
      }
      delay(300);

   // if not connected - making own AP  
  if (WiFi.status() != WL_CONNECTED){
    WiFi.disconnect();
    
    delay(1000);
    // Replace with your network credentials
    const char* ssid2     = "BB.LAB.ARON";
    const char* password2 = "123456789";

   /* Put IP Address details */
    IPAddress local_ip(192,168,1,1);
    IPAddress gateway(192,168,1,1);
    IPAddress subnet(255,255,255,0);


    
    Serial.print("Setting AP (Access Point)…");
    WiFi.softAP(ssid2, password2);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(300);
    Serial.println(WiFi.softAPIP());
  }
  else
 {
  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());
 }

  

  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send_P(200, "text/html", index_html, processor);
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/serial", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/serial.html", "text/html");
  });
   server.on("/img", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/aronmk2.jpg", "image/jpeg");
  });
 

  // Start server
  server.begin();


  Serial.print("Starting up");
  for (int k=0; k < 21; k+=4){
    digitalWrite(LEDPIN, HIGH);
    Serial.print(".");
    delay(400 - k*20);
    digitalWrite(LEDPIN, LOW);
    delay(400 - k*20);
  }
  Serial.println();
  digitalWrite(LEDPIN, HIGH);
}

// simple loop moving test - usefull for hardware verification.
const int sequences = 1;
const int max_steps = 30;
int test_step = 0;
int set_sequence = 0;
const int test_steps[sequences] = {2};

int test_pos[sequences][max_steps][servos] = {
  { // first sequence
  {70,70,110,110,110,110,70,70,140,80,90,40,90,90,90,90},
  {110,110,70,70,70,70,110,110,100,40,130,80,90,90,90,90}
  }
};


void loop() {
// marking current time
  now = millis();
// making the read from serial
  sCmd.readSerial();
// checking the `webSocket stuff
  ws.cleanupClients();

  
// testing servo position loop.
if (move_done && in_loop) {
  
  DEBUG_PRINTLN("Move is done...");
  DEBUG_PRINT("Test step ");
  DEBUG_PRINT(test_step);
  DEBUG_PRINT(" set angle ");
  DEBUG_PRINTLN(test_pos[test_step]);
  
  for (int s=0; s < servos; s++){
    servo_target[s] = test_pos[set_sequence][test_step][s];
  }
  test_step++;
  
  if (test_step == test_steps[set_sequence]){
    test_step = 0;
  }
} // end of servo testing



  
// making the servos moves - the main loop action
if (now > last + servo_delay){
  last = now;  
  move_done = true;
  for (int s=0; s < servos; s++){

    // checking if need to move
    if (abs(servo_target[s] - servo_current[s]) > 1){
      // if the servo is away more than a 1 degree we move
      move_done = false;
      if (in_linear){
        // procedure for the linera move with constant step
        // figuring out the direction
        float the_step = 0.0;
        if (servo_current[s] < servo_target[s] - servo_step[s]){
            the_step = servo_step[s];
        }
        else if (servo_current[s] > servo_target[s] + servo_step[s])
        {
            the_step = -servo_step[s];
        } 
        else 
        {
            the_step = servo_target[s] - servo_current[s];
        }
        
        servo_current[s] += the_step;
      }
      else{
        servo_current[s] = servo_ramp[s] * servo_current[s] + (1 - servo_ramp[s]) * servo_target[s];
      }
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
} //set_servos


void fakeData(){
  // introducing the fake data - for the sake of testing
  // getting the param
  
  char chars[64];

  sprintf(chars, "circ %d %d\n", 10, 30);
  
  sCmd.readStr(chars);
}

void command(char *cmd){
  Serial.println(cmd);
  sCmd.readStr(cmd);
}

void toogleMmode(){
  // switching the move interpolation mode

  int aNumber;
  char *arg;
  
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      if (aNumber > 0) {
        in_linear = true;
        }
      else {
        in_linear = false;
      }
    } else {
      in_linear = !in_linear;
    }
    Serial.println(in_linear);

}

void lookAround(){
  // looking around in LR and UPDN 
  int aNumber[2];
  char *arg;
  int arg_num = 0;
  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber[a] = atoi(arg);    // Converts a char string to int
      arg_num++;
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
      break;
    }
  }

  if (arg_num == 2){
    // making legs up down  

      aNumber[0] = -1*constrain(aNumber[0], -50,50);
      aNumber[1] = -1*constrain(aNumber[1], -50,50);
      
      // the up down pose
        servo_up[0] = -aNumber[1];
        servo_up[2] = -aNumber[1];
        
        servo_up[4] = aNumber[1];
        servo_up[6] = aNumber[1];
        
      // left right
      // it's just like the swing move
      servo_lr[8]  =  aNumber[0] / 2;
      servo_lr[9]  =  aNumber[0] / 2;
      servo_lr[10] =   aNumber[0];
      servo_lr[11] =   aNumber[0];

      
      // servo_lr[0] = -aNumber[0];
      // servo_lr[1] = -aNumber[0];

      // servo_lr[2] = aNumber[0];
      // servo_lr[3] = aNumber[0];

      // servo_lr[4] = aNumber[0];
      // servo_lr[5] = aNumber[0];

      // servo_lr[6] = -aNumber[0];
      // servo_lr[7] = -aNumber[0];

      // executing the changes
      makeMove();
      
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
    }
  
} // makeUp


void makeMove(){
  // the incremental model moves
  for (int s=0; s<servos; s++){
    servo_target[s]  = servo_home[s];
    servo_target[s] += servo_up[s];
    servo_target[s] += servo_fb[s];
    servo_target[s] += servo_lr[s];

    servo_target[s] += servo_legs[s];
    }
  
  } // makeMove

void makeUp(){
  // rising or lowering all legs. 
  int aNumber[2];
  char *arg;
  int arg_num = 0;
  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber[a] = atoi(arg);    // Converts a char string to int
      arg_num++;
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
      break;
    }
  }

  if (arg_num == 2){
    // making legs up down  

      aNumber[0] = constrain(aNumber[0], -40,40);
      aNumber[1] = constrain(aNumber[1], -40,40);
      
      
        servo_up[0] = -aNumber[1];
        servo_up[2] = -aNumber[1];
        
        servo_up[4] = -aNumber[0];
        servo_up[6] = -aNumber[0];
        
      
      
//      float aNumberF = -1.0 * (aNumber / 3.5); // to compensate the front back when we rise or lower
//      aNumber = (int) aNumberF;
//      DEBUG_PRINTLN(aNumber);
//      for (int s=1; s < 8; s+=2) {
//      //  servo_target[s] = servo_target[s] - aNumber + (servo_zero[s] - servo_target[s]);
//        servo_up[s] = -aNumber;
//      }

      makeMove();
      
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
    }
  
} // makeUp


void setStepSize(){
  // making a Twist move - may be dangerous!! 
  int aNumber;
  char *arg;
  
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      DEBUG_PRINT("Twist by ");
      DEBUG_PRINT(aNumber);
      DEBUG_PRINTLN(" degrees"); 

      aNumber = constrain(aNumber, 1,30);
      float new_step = aNumber * 0.1;
      Serial.println(new_step);
      for (int s=0; s<servos; s++){
        servo_step[s] = new_step;
      }
      
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
    }
  
} // 


void setDelay(){
  // making a Twist move - may be dangerous!! 
  int aNumber;
  char *arg;
  
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      aNumber = constrain(aNumber, 10,1000);
      servo_delay = aNumber;
      
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
    }
  
} // makeTwist

void makeTwist(){
  // making a Twist move - may be dangerous!! 
  int aNumber;
  char *arg;
  
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber = atoi(arg);    // Converts a char string to an integer
      DEBUG_PRINT("Twist by ");
      DEBUG_PRINT(aNumber);
      DEBUG_PRINTLN(" degrees"); 

      aNumber = constrain(aNumber, -25,25);
      // modyfing the lr setting
      servo_lr[8]  =  aNumber;
      servo_lr[9]  =  aNumber;
      servo_lr[10] =  -aNumber;
      servo_lr[11] =  -aNumber;

      
      servo_lr[0] = -aNumber;
      servo_lr[1] = -aNumber;

      servo_lr[2] = aNumber;
      servo_lr[3] = aNumber;

      servo_lr[4] = aNumber;
      servo_lr[5] = aNumber;

      servo_lr[6] = -aNumber;
      servo_lr[7] = -aNumber;
      

      makeMove();
      
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
    }
  
} // makeTwist


void moveLeg() {
  // Function for recieve data for single leg move in incremental model
  
  int theValue[4];
  int arg_num = 0;
  char *arg;

  
  for (int a=0; a<4; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      theValue[a] = atoi(arg);    // Converts a char string to an integer
      arg_num++;
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK  4 args expected");
      break;
    }
  }
  
  if (arg_num == 4) {
    int leg = theValue[0];
    int s1,s2,s3;

    switch (leg) {
      case 1:
        s1 = 0;
        s2 = 1;
        s3 = 8;
        break;

      case 2:
        s1 = 2;
        s2 = 3;
        s3 = 9;
        break;

      case 3:
        s1 = 4;
        s2 = 5;
        s3 = 10;
        break;

      case 4:
        s1 = 6;
        s2 = 7;
        s3 = 11;
        break;
      }

    // values are like 1 - up/dn, 2 - f/b, 3 - l/r 
    servo_legs[s1]  = theValue[1];
    
    servo_legs[s1] += theValue[2];
    servo_legs[s2]  = theValue[2];

    servo_legs[s3] = theValue[3];

    // making the move
    makeMove();
  }
} // moveLeg






void togglePower(){
  // toggle power for the servos
  power = !power;
  digitalWrite(POWEROUT, power);
}

void makeZero(){
  // setting up the home to the zero positions
  for (int s=0; s<servos; s++){ 
    servo_home[s] = servo_zero[s];
    servo_up[s] = 0;
    servo_fb[s] = 0;
    servo_lr[s] = 0;
    servo_legs[s] = 0;
  }
}

void makeHoming(){
  // this one stops tle loping and set initial positinos
  in_loop = false;
  DEBUG_PRINT("Homming ");
  for (int s=0; s < servos; s++){
      servo_target[s] = servo_home[s];
      DEBUG_PRINT(".");
    }
   DEBUG_PRINTLN();
  }

void runTest(){
  // this one toggle the looping of the test set.
  in_loop = !in_loop;
  if (in_loop){
    DEBUG_PRINTLN("Running loop...");
    }
  else {
    DEBUG_PRINTLN("Stopping loop!");
  }
}

void allStop(){
  for (int n=0; n<4; n++){
    ledcWrite(n,0);
  }
}

void driveWheels() {
  // protype function to recieve ramp and set it for all servos
  
  int aNumber[2];
  char *arg;
  int arg_num = 0;
  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber[a] = atoi(arg);    // Converts a char string to int
      arg_num++;
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
      break;
    }
  }

  if (arg_num == 2){
    // making the wheel drive
    drive(aNumber[0], aNumber[1]);
    
    
    
  }
  
} // driveWheels

void drive(int left, int right){
  // running the motors
   left  *= -1;
   right *= -1;

      if (left < 0){
        ledcWrite(0, abs(left));
        ledcWrite(1, 0);
      }
      else {
        ledcWrite(1, abs(left));
        ledcWrite(0, 0);
      }

      if (right < 0){
        ledcWrite(2, abs(right));
        ledcWrite(3, 0);
      }
      else {
        ledcWrite(3, abs(right));
        ledcWrite(2, 0);
      } 
  
} // drive

void makeWalk(){
  
 int aNumber[2];
  char *arg;
  int arg_num = 0;
  
  for (int a=0; a<2; a++){
    // shifting the index
    arg = sCmd.next();
    
    if (arg != NULL) {
      aNumber[a] = atoi(arg);    // Converts a char string to int
      arg_num++;
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
      break;
    }
  }

  if (arg_num == 2){
    // making the wheel drive
    Walk(aNumber[0], aNumber[1]);
  }  
} // makeWalk

void Walk(int spd, int dir){
  // making aron sudo walk
  // spd will determine the motors drive speed and the ramp
  // dir will make the motors direction

  dir = constrain(dir, -150, 150);
  spd = constrain(spd, -150, 150);

  if (abs(spd) < 10) spd = 0;
  if (abs(dir) < 10) dir = 0;

  int left  = spd + dir;
  int right = spd - dir;
  
  if ( left < 0) {
    left -= 40;
  } else if (left > 0) left +=40;
  
  if (right < 0) {
    right -= 40;
  } else if (right > 0) right += 40;

  
//  Serial.println(left);
//  Serial.println(right);
  if (abs(left) > 50 || abs(right) > 50){
    
    float newramp = map((abs(left)+abs(right)) / 2, 40,200, 990, 900);
//    Serial.println(newramp);
    newramp /= 1000;
//    Serial.println(newramp);
    
    for( int s=0; s < servos; s++) servo_ramp[s]= newramp;  
    left  *= 0.75;
    right *= 0.75;
    drive(left, right);
    in_loop = true;
    }
    else {
      in_loop = false;
      drive(0,0);
      command("twist 0\n");
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
      DEBUG_PRINT("ramp ");
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(ramp);

      for( int s=0; s < servos; s++) servo_ramp[s]= ramp;
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK no ramp argument");
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
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(theValue[a]);
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
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK  2 args expected");
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
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK  16 args expected");
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
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK  2 args expected");
      break;
    }
  }
  
  if (arg_num == 2) {
    // we move all sevos by this delta
      for (int s=0; s < servos; s++){  
        if (s > 7) {
          servo_lr[s] = theValue[1];
        } else {
          servo_fb[s] = theValue[0];  
        }  
    }
  makeMove();
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
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(theValue[a]);
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK  16 args expected");
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
    if (s < servos-1) Serial.print(" ");
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
      DEBUG_PRINT("Swing by ");
      DEBUG_PRINT(aNumber);
      DEBUG_PRINTLN(" degrees"); 

      aNumber = constrain(aNumber, -70,70);
      for (int s=8; s < 12; s++) servo_target[s] = servo_home[s] + aNumber;
      
    }
    else {
      // we escape as no argument was found. 
      DEBUG_PRINTLN("NOK");
    }
  
} // makeSwing



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
      DEBUG_PRINT("argument ");
      DEBUG_PRINT(a);
      DEBUG_PRINT(" = ");
      DEBUG_PRINTLN(aNumber);
    }
    else {
      // we escape as no argument was found. 
      break;
    }
  }
  
DEBUG_PRINT("Recieved ");
DEBUG_PRINT(arg_num);
DEBUG_PRINTLN(" arguments");
  
} // processCommands



// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("NOK");
}

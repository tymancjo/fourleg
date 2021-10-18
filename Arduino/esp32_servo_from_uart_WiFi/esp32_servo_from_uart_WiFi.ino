// # FLR - reciver module
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
// Load Wi-Fi library
#include <WiFi.h>
#include <WebServer.h>

// Stuff for serial communication use
bool newData = false;
bool fakeData = false;
const byte numChars = 48;
char receivedChars[numChars];
char tempChars[numChars];
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
// global variables to keep the received data
int param[8];
int command;

// Stuff for wifi
// Replace with your network credentials
const char* ssid     = "BB.LAB.ARON";
const char* password = "123456789";

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// Set web server port number to 80
WebServer server(80);


// kicking of the pwm module definition
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// defining the values used for PWM servo control
// those were selected after experiment with the real used hw servos
// #define SERVOMIN 160  // This is the 'minimum' pulse length count (out of 4096)
// #define SERVOMAX 505  // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN 170  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 495  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// for servos attached to the PWM module
// initial reset position
const int pos0 = 0;
// total servo canals count
const int servo_count = 16;
float ramp = 0.95;

uint8_t servos_pos[servo_count] = {90};

float servos_curr[servo_count] = {90};
bool servos_interpolate[servo_count] = {true};

unsigned long now;
unsigned long last = 0;

// Stuff for pre-programmed poses
const int poses_rows = 20;

int poses[poses_rows][8] = {
  {180, 163, 10, 17, 180, 153, 0, 17},
  {148, 51, 42, 129, 148, 41, 32, 129},
  {64, 79, 126, 101, 64, 69, 116, 101},
  {180, 122, 10, 58, 64, 69, 116, 101},
  {148, 51, 42, 129, 148, 41, 0, 0},
  {148, 51, 42, 129, 180, 180, 32, 129}
};

String pose_names[poses_rows] = {
"LEZEC",
"JAMNIK",
"TROP",
"SIAD",
"Lapka L",
"Lapka R",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos",
"Pos"
};

int total_poses = 6;

// handling the move sequences.
bool move_done = false;
bool in_sequence = false;
int sequence_nr = 0;
int sequence_rep = 1;
int sequence_crep = 0;
int sequence_step = 0;

const uint8_t max_sequences = 1;
const uint8_t max_steps = 30;
const uint8_t max_servos = 8;

int sequences[max_sequences][max_steps][max_servos] = 
{
    {
      {91,61,99,119,91,51,89,119},
      {110,104,99,119,91,51,89,119},
      {110,104,74,116,116,54,64,116},
      {95,116,64,119,116,54,64,116},
      {95,116,64,119,109,130,64,116},
      {95,116,64,119,129,113,64,116},
      {136,95,64,119,129,113,64,116},
      {136,95,64,119,129,113,43,42},
      {136,95,64,119,129,113,79,56},
      {115,122,64,119,115,112,79,56},
      {61,63,125,117,51,43,129,127},
      {61,63,86,93,51,43,129,127},
      {61,63,86,93,105,23,75,147},
      {99,92,86,93,105,23,75,147},
      {91,61,99,119,91,51,89,119}
    },
};

float ramps[max_steps] = {0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 0.89, 0.95, 0.9, 0.85, 0.85, 0.85};
uint8_t sequence_steps[max_sequences] = {15};

void setup()
{
  // kicking off serial
  Serial.begin(115200);

  // Servos reset
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  delay(100);

  Serial.print("Resetting Servo: ");
  for (int i = 0; i < servo_count; i++)
  {
    servos_pos[i] = 90;
    servos_curr[i] = servos_pos[i];
    Serial.print(i);
    Serial.print(" ");
    Serial.println(servos_curr[i]);
    pwm.setPWM(i, 0, (int)map(servos_curr[i], 0, 180, SERVOMIN, SERVOMAX));
    delay(10);
  }
  delay(500);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  
  
  server.on("/p", handle_pose);

  server.on("/s", handle_sequence);
    
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");

  // the sequences data

// the crazy dog step
  sequence_steps[0] = 15;
}


void loop() {

  // serving the www
  server.handleClient();

  now = millis();

  // grabstuff from serial
  recvWithStartEndMarkers();

  // if we have some new data...
  if (newData || fakeData)
  {
    if (newData){
      strcpy(tempChars, receivedChars);
      parseData();
    }

    newData = false;
    fakeData = false;

    // setting the new position of servo
    if (command == 42) {
      for (int ss = 0; ss < 8; ss++) {
        if (param[ss] > -1 && param[ss] < 181) {
          servos_pos[ss] = param[ss];
        }
      }
    }

    if (command == 44) {
      // bringing in new pose from the stored ones
      int new_pose = param[0];
      if (new_pose < total_poses) {
        // if the selected pose is in the range - we read the values
        for (int ss = 0; ss < 8; ss++) {
          if (poses[new_pose][ss] > -1 && poses[new_pose][ss] < 181) {
          servos_pos[ss] = poses[new_pose][ss];
          }
        }
      }
    }


    if (command == 55){
      // storing new pose
      if (total_poses < poses_rows){
        // we have a place for the new pose
        int new_pose = total_poses;
        total_poses++;
        
          for (int ss = 0; ss < 8; ss++) {
            poses[new_pose][ss] = param[ss];
            Serial.println(poses[new_pose][ss]);
          }

        Serial.print("new pose at: ");
        Serial.println(new_pose);
        }
      }

    if (command == 45){
      // option to change the ramp
      ramp = param[0] / 100.0;
      }
    
  }

  // handling the servo slope moving
  // every 10 ms we do a step
  if (now > last + 10) {
    last = now;
    move_done = true;

    for (uint8_t s = 0; s < servo_count; s++) {
      if  (abs(servos_curr[s] - servos_pos[s]) > 1) {
        // if need to move more then 1 degree
        move_done = false; // since we move we are not done yet...
        servos_curr[s] = (ramp * servos_curr[s] + (1-ramp) * servos_pos[s]);
        pwm.setPWM(s, 0, (int)map(servos_curr[s], 0, 180, SERVOMIN, SERVOMAX));
      }
    }
  

  if (in_sequence and move_done){
    // if we are in the sequence we are making the next step when previous is done:
    Serial.print("Seq active ");
    Serial.println(sequence_nr);
    ramp = 0.85;
    if (sequence_step < sequence_steps[sequence_nr]-1) {
      // we still have some steps to do
      sequence_step++; // taking the step count up
      Serial.print("step ");
      Serial.print(sequence_step);
      Serial.print(" of ");
      Serial.println(sequence_steps[sequence_nr]);
      ramp = ramps[sequence_step];
      
      for (int s=0; s<max_servos; s++){
        // reading the servos positions from sequence step
        param[s] = sequences[sequence_nr][sequence_step][s];
        }
        // faking new data from serial
          command = 42;
          fakeData = true;
          
      } else 
        {
        // we are done with the sequence and reset the states
        if (sequence_crep < sequence_rep){
          sequence_crep++;
          Serial.println("Next round...");
          }
        else {
          in_sequence = false;
          ramp = 0.95;
          sequence_crep = 0;
          Serial.println("Seq.Done");
          }
        
        sequence_step = -1;
        
        }
  }}
      

  } // loop()






  void recvWithStartEndMarkers()
  {
    //  Read data in this style <C, 1, 2, 3, 4, 5, 6, 7, 8>
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false)
    {
      rc = Serial.read();

      if (recvInProgress == true)
      {
        if (rc != endMarker)
        {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars)
          {
            ndx = numChars - 1;
          }
        }
        else
        {
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }

      else if (rc == startMarker)
      {
        recvInProgress = true;
      }
    }
  }

  void parseData()
  { // split the data into its parts

    char *strtokIndx; // this is used by strtok() as an index

    Serial.println(tempChars);

    strtokIndx = strtok(tempChars, ","); // get the first part - the string
    command = atoi(strtokIndx);          // convert this part to an int
    Serial.println(command);

    strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, ",");
    param[0] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[1] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[2] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[3] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[4] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[5] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[6] = (int)atof(strtokIndx); // convert this part to a int

    strtokIndx = strtok(NULL, ",");
    param[7] = (int)atof(strtokIndx); // convert this part to a int
  }

void handle_OnConnect() {
  
  server.send(200, "text/html", SendHTML()); 
  
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(){
  
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>LED Control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";

  for (int s=0; s < total_poses; s++){
    ptr += "<p><a href=/p?p=";
    ptr += s;
    ptr += ">";
    ptr += s;
    ptr += " ";
    ptr += pose_names[s];
    ptr += " </a></p>";
    
   }

  ptr += "<h3>Sequences</h3>";

  for (int s=0; s < max_sequences; s++){
    ptr += "<p><a href=/s?k=1&s=";
    ptr += s;
    ptr += ">";
    ptr += s;
    ptr += " :sequence";
    ptr += "</a></p>";
    }
    for (int s=0; s < max_sequences; s++){
    ptr += "<p><a href=/s?k=5&s=";
    ptr += s;
    ptr += ">";
    ptr += s;
    ptr += " :sequence x5";
    ptr += "</a></p>";
    }
   
  ptr +="</body>\n";
  ptr +="</html>\n";
  
  return ptr;
}

void handle_pose(){
  int pose = server.arg("p").toInt();
  command = 44;
  param[0] = pose;
  fakeData = true;
  server.send(200, "text/html", SendHTML());
}

void handle_sequence(){
  int s = server.arg("s").toInt();
  int k = server.arg("k").toInt();
  
  if (k <= 0 || k > 20){
    k = 1;
    }
    
  if (s > -1 && s < max_sequences && !in_sequence){
      // if recieved data makes sense
      sequence_nr = s;
      sequence_rep = k;
      in_sequence = true;
      Serial.print("Start sequence ");
      Serial.println(sequence_nr);
      server.send(200, "text/html", SendHTML());
    }
  
  }

// # FLR - reciver module
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

// Stuff for serial communication use
bool newData = false;
const byte numChars = 48;
char receivedChars[numChars];
char tempChars[numChars];
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
// global variables to keep the received data
int param[8];
int command;


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

uint8_t servos_pos[servo_count] = {90};

float servos_curr[servo_count] = {90};
bool servos_interpolate[servo_count] = {true};

unsigned long now;
unsigned long last = 0;

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

  delay(1000);
}

void loop() {

  now = millis();

  // grabstuff from serial
  recvWithStartEndMarkers();

  // if we have some new data...
  if (newData == true)
  {
    strcpy(tempChars, receivedChars);
    parseData();

    newData = false;
    // Serial.println("Data sent to CAN");

    // setting the new position of servo
    if (command == 42) {
      for (int ss = 0; ss < 8; ss++) {
        if (param[ss] > -1 && param[ss] < 181){
        servos_pos[ss] = param[ss];
        }
      }
    }
  }

  // handling the servo slope moving
  if (now > last + 10) {
    last = now;

    for (uint8_t s = 0; s < servo_count; s++) {
      if  (abs(servos_curr[s] - servos_pos[s]) > 1) {

        servos_curr[s] = (0.95 * servos_curr[s] + 0.05 * servos_pos[s]);
        pwm.setPWM(s, 0, (int)map(servos_curr[s], 0, 180, SERVOMIN, SERVOMAX));
      }
    }
  }

}






void recvWithStartEndMarkers()
{
  //  Read data in this style <C, 1, 2, 3, 4, 5>
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

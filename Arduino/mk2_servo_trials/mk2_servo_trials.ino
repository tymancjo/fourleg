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


float servo_target[servos];
float servo_current[servos];
bool    servo_inverse[servos] = { 0,0,1,1,0,0,1,1, 0,0,1,1,0,0,0,0 };
float   servo_ramp[servos];

bool move_done = false;
int waiting = 0;

unsigned long now = millis();
unsigned long last = millis();
unsigned long last_step = millis();



void setup() {
  Serial.begin(115200);
  Serial.println("Aroin mk2 Servo test!");

  // preparing the inital positions.
  for (int k = 0; k < servos; k++) {
    servo_target[k] = 90;
    servo_current[k] = 80;
    // servo_inverse[k] = false;
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

  delay(5000);
  Serial.println("Starting up...");
}

int test_step = 0;
const int test_steps = 3;
int test_pos[test_steps] = {90, 70, 110};
//int test_pos[test_steps] = {90, 90, 90};

void loop() {
// marking time
now = millis();

  
// testing servo position
if (move_done) {
  
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

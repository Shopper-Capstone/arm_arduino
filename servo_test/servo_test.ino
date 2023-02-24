#include <Adafruit_PWMServoDriver.h>

/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

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

// Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

typedef struct {
  int servonum; // Pin

  double servo_min;
  double servo_max;
  double angle_min;
  double angle_max;

  int current_pulse_width;
} servo_t;

servo_t base;
servo_t shoulder;
servo_t elbow;
servo_t wrist;
servo_t gripper;

int pulseWidth(servo_t *servo, int angle)
{
  if(angle > servo->angle_max) {
    return servo->servo_max;
  } else if(angle < servo->angle_min) {
    return servo->servo_min;
  }

  // return (int)(SERVOMIN + (SERVOMAX - SERVOMIN)/(ANGLEMAX - ANGLEMIN)*angle);
  return (int)(servo->servo_min + (servo->servo_max - servo->servo_min)/(servo->angle_max - servo->angle_min)*angle);
}

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  base.servonum = 0;
  base.servo_min = 103.0;
  base.servo_max = 512.0;
  base.angle_min = 0.0;
  base.angle_max = 180.0;
  base.current_pulse_width = base.servo_min;

  shoulder.servonum = 2;
  shoulder.servo_min = 130.0;
  shoulder.servo_max = 460.0;
  shoulder.angle_min = 0.0;
  shoulder.angle_max = 180.0;
  shoulder.current_pulse_width = shoulder.servo_min;

  elbow.servonum = 4;
  elbow.servo_min = 115.0;
  elbow.servo_max = 510.0;
  elbow.angle_min = 0.0;
  elbow.angle_max = 180.0;
  elbow.current_pulse_width = elbow.servo_min;

  wrist.servonum = 6;
  wrist.servo_min = 255.0;
  wrist.servo_max = 460.0;
  wrist.angle_min = 0.0;
  wrist.angle_max = 90.0;
  wrist.current_pulse_width = wrist.servo_min;

  gripper.servonum = 8;
  gripper.servo_min = 150.0;
  gripper.servo_max = 430.0;
  gripper.angle_min = 0.0;
  gripper.angle_max = 110.0;
  gripper.current_pulse_width = gripper.servo_min;

  pwm.begin();
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

  delay(10);

  // Resets back to 0 degrees
  pwm.setPWM(base.servonum,     0, base.servo_min);
  pwm.setPWM(shoulder.servonum, 0, base.servo_min);
  pwm.setPWM(elbow.servonum,    0, base.servo_min);
  pwm.setPWM(wrist.servonum,    0, base.servo_min);
  pwm.setPWM(gripper.servonum,  0, base.servo_min);

  // moveToAngle(&shoulder, 0);
  // delay(1000);
  // moveToAngle(&shoulder, 180);
  // delay(1000);
  // // moveToAngle(&shoulder, 90);
  // // delay(1000);
  // pwm.setPWM(shoulder.servonum, 0, 300);
}

void moveToAngle(servo_t *servo, int angle)
{
  int final_pulse_width = pulseWidth(servo, angle);
  Serial.print("Servo: ");
  Serial.println(servo->servonum);
  Serial.print("Pulse width for ");
  Serial.print(angle);
  Serial.print(" degrees: ");
  Serial.println(final_pulse_width);

  if(final_pulse_width < servo->current_pulse_width) {
    for (uint16_t pulselen_to_write = servo->current_pulse_width; pulselen_to_write > final_pulse_width; pulselen_to_write--) {
      pwm.setPWM(servo->servonum, 0, pulselen_to_write);
    }
  } else { // final_pulse_width > current_pulse_width
    for (uint16_t pulselen_to_write = servo->current_pulse_width; pulselen_to_write < final_pulse_width; pulselen_to_write++) {
      pwm.setPWM(servo->servonum, 0, pulselen_to_write);
    }
  }

  servo->current_pulse_width = final_pulse_width;
}

void loop() {
  int testangle;

//  if(increasing) {
//    testangle += 20;
//    if(testangle > 180) {
//      testangle = 180;
//      increasing = false;
//    }
//  } else {
//    testangle -= 20;
//    if(testangle < 0) {
//      testangle = 0;
//      increasing = true;
//    }
//  }

  // int angles[] = {30, 91, 40, 152, 55, 149, 115, 8, 50, 158}; // base
  // int angles[] = {92, 99, 80, 61, 73, 108, 93, 94, 64, 115}; // shoulder/elbow
  int angles[] = {10, 5, 41, 13, 44, 12, 25, 62, 86, 5}; // wrist
  for(int i = 0; i < 10; i++) {
    testangle = angles[i];
    moveToAngle(&wrist, testangle);
    delay(2000);
  }
}

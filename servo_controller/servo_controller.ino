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

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#include "Arduino.h"
#include "ArduinoHardware.h"
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

// Set ROS - handler, subscribe message, publish message (debugging)
ros::NodeHandle  nh;
std_msgs::Int16MultiArray str_msg2;
ros::Publisher chatter("servoarm", &str_msg2);
int servoDegree[6];

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
  return (int)(servo->servo_min + (servo->servo_max - servo->servo_min)/(servo->angle_max - servo->angle_min)*(angle - servo->angle_min));
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup() {
  // Setup ROS for subscribe and publish
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
//  Serial.begin(9600);
//  Serial.println("ROS Arduino comms!");

  base.servonum = 0;
  base.servo_min = 130.0;
  base.servo_max = 512.0;
  base.angle_min = 0.0;
  base.angle_max = 180.0;
  base.current_pulse_width = base.servo_min;

  shoulder.servonum = 3;
  shoulder.servo_min = 120.0;
  shoulder.servo_max = 430.0;
  shoulder.angle_min = 0.0;
  shoulder.angle_max = 180.0;
  shoulder.current_pulse_width = shoulder.servo_min;

  elbow.servonum = 15;
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
  gripper.angle_min = 0.0; // open
  gripper.angle_max = 110.0; // closed
  gripper.current_pulse_width = gripper.servo_min;


  // Setup PWM Controller object
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
  pwm.setPWM(shoulder.servonum, 0, shoulder.servo_min);
  pwm.setPWM(elbow.servonum,    0, elbow.servo_min);
  pwm.setPWM(wrist.servonum,    0, wrist.servo_min);
  pwm.setPWM(gripper.servonum,  0, gripper.servo_min);

  moveToAngle(&base, 90);
  delay(1000);
  moveToAngle(&shoulder, 90);
  delay(1000);
  moveToAngle(&elbow, 90);
  delay(1000);
  moveToAngle(&wrist, 90);
  delay(1000);
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

// Convert radians to degreees
double radiansToDegrees(float position_radians) {
  position_radians = position_radians * 57.2958;
  return position_radians;
}

// Sometimes servo angle goes just above 180 or just below 0 - trim to 0 or 180
double trimLimits(double mtr_pos) {
  if(mtr_pos > 180) {
    mtr_pos = 180;
  }
  if(mtr_pos < 0) {
    mtr_pos = 0;
  }
  return mtr_pos;
}  

double gripper_position(float position_m)
{
  if(position_m < 0.005) { // open
    return (gripper.angle_min);
  } 
  return 100; // close
}

// Function move motor to ROS angle
void servo_cb(const sensor_msgs::JointState& cmd_msg) {
  // these can have adjustment factors + or - 90/180/whatever degrees
  double mtrDegreeBase = trimLimits(radiansToDegrees(cmd_msg.position[0]) + 90);
  double mtrDegreeShoulder = trimLimits(radiansToDegrees(cmd_msg.position[1]) + 90);
  double mtrDegreeElbow = trimLimits(radiansToDegrees(cmd_msg.position[2]) + 45);
  double mtrDegreeWrist = trimLimits(radiansToDegrees(cmd_msg.position[3]) + 45);
  double mtrDegreePivot = trimLimits(radiansToDegrees(cmd_msg.position[5]));
  double mtrDegreeGripper = trimLimits(gripper_position(cmd_msg.position[4]));

  // Store motor movements for publishing back to ROS (debugging)
  servoDegree[0] = mtrDegreeBase - 90;
  servoDegree[1] = mtrDegreeShoulder - 90;
  servoDegree[2] = mtrDegreeElbow - 45;
  servoDegree[3] = mtrDegreeWrist - 45;
  servoDegree[5] = mtrDegreePivot;
  servoDegree[4] = cmd_msg.position[4];

//  moveMotorDeg(mtrDegreeBase, motorBase);
  moveToAngle(&base, mtrDegreeBase);
  moveToAngle(&shoulder, 180 - (mtrDegreeShoulder));
  moveToAngle(&elbow, (mtrDegreeElbow));
  moveToAngle(&wrist, mtrDegreeWrist);
  moveToAngle(&gripper, mtrDegreeGripper);
}


void loop() {
  str_msg2.data = servoDegree;
  str_msg2.data_length = 6;
  chatter.publish( &str_msg2 );
  nh.spinOnce();
  delay(1);
}

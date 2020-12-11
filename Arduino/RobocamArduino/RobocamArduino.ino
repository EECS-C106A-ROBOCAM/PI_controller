/**
   Control the ROBOCAM arm based on inputs received over ROS Serial.
   @author Jaiveer Singh (j.singh@berkeley.edu)
   Copyright 2020.
*/

#include <Arduino.h>

#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;

Servo baseYawServo;
Servo bot4BServo;
Servo top4BServo;
Servo camPitchServo;
Servo camYawServo;
Servo camRollServo;

const short PIN_BASE_YAW = 0;
const short PIN_BOT_4B = 0;
const short PIN_TOP_4B = 0;
const short PIN_CAM_PITCH = 0;
const short PIN_CAM_YAW = 0;
const short PIN_CAM_ROLL = 0;

const char ROS_TOPIC_NAME[] = "/arduino_joints";


/**
   Checks if motor angles are all in range.
   @param motorAnglesArray Length 6 array in base-to-tool order
   @return true if motor angles are in range and valid; false otherwise
*/
bool motorAnglesInRange(short* motorAnglesArray) {
  // Base Yaw Servo
  const short BASE_YAW_MIN = 0;
  const short BASE_YAW_MAX = 0;

  // Bottom 4 Bar Servo
  const short BOT_4B_MIN = 0;
  const short BOT_4B_MAX = 0;

  // Top 4 Bar Servo
  const short TOP_4B_MIN = 0;
  const short TOP_4B_MAX = 0;

  // Camera Pitch Servo
  const short CAM_PITCH_MIN = 0;
  const short CAM_PITCH_MAX = 0;

  // Camera Yaw Servo
  const short CAM_YAW_MIN = 0;
  const short CAM_YAW_MAX = 0;

  // Camera Roll Servo
  const short CAM_ROLL_MIN = 0;
  const short CAM_ROLL_MAX = 0;

  short MIN_ANGLES[6] = { BASE_YAW_MIN, BOT_4B_MIN, TOP_4B_MIN, CAM_PITCH_MIN, CAM_YAW_MIN, CAM_ROLL_MIN};
  short MAX_ANGLES[6] = { BASE_YAW_MAX, BOT_4B_MAX, TOP_4B_MAX, CAM_PITCH_MAX, CAM_YAW_MAX, CAM_ROLL_MAX};

  bool success = true;
  for (int i = 0; i < 6; ++i) {
    success &= (motorAnglesArray[i] >= MIN_ANGLES[i]) && (motorAnglesArray[i] <= MAX_ANGLES[i]);
  }
  return success;
}

/**
   Converts the 6 joint inputs into 6 motor angles.
   @param jointInputArray Length 6 array of joint input angles
   @param motorOutputArray Length 6 array of motor output angles
   @return true if motor angles are in range and valid; false otherwise
*/
bool jointToMotorAngles(float* jointInputArray, short* motorOutputArray) {
  for (int i = 0; i < 6; ++i) {
    motorOutputArray[i] = jointInputArray[i]; // Narrowing from float to short
  }

  return motorAnglesInRange(motorOutputArray);
}

/**
   Actuate the motors to the specified angles.
   @param motorAngles Length 6 array of motor angles in base-to-tool order
   @param mode Specific completion mode to use:
                - 'i': [I]nstantly move all joints
                - 'a': [A]daptively move joints with big deltas slowly, small deltas instantly
                - 's': [S]ynchronize all joints to complete after specified duration
   @param duration How long a synchronized move should take
   @param deltaThreshold How large a delta needs to be before we adaptively slow the movement
   @param stepDelay How many milliseconds to wait between step updates
*/
bool actuateMotors(short* motorAngles, char mode = 'i', int duration = 500, int deltaThreshold = 10, int stepDelay = 10) {
  if (!motorAnglesInRange(motorAngles)) {
    // Error: the motor angles are out of range!
    // TODO(JS): Some kind of error sent over ROS
    return false;
  }

  switch (mode) {
    case 'i': {
        baseYawServo.write(motorAngles[0]);
        bot4BServo.write(motorAngles[1]);
        top4BServo.write(motorAngles[2]);
        camPitchServo.write(motorAngles[3]);
        camYawServo.write(motorAngles[4]);
        camRollServo.write(motorAngles[5]);

        return true;
      } break;

    case 'a':
    case 's': {
        // Error: Not implemented yet!
        // TODO(JS): Implement these
        return false;
      } break;

    default: {
        // Error: Unknown mode!
        // TODO(JS): Log error somehow
        return false;
      } break;
  }


  return true;
}

/**
   Handles main Arduino callback to actuate servo motors.
   @param jointStateMsg ROS JointState message containing 6 angles
*/
void motorCallback(const sensor_msgs::JointState& jointStateMsg) {
  float* jointAngles = jointStateMsg.position;
  short motorAngles[6];
  if (!jointToMotorAngles(jointAngles, motorAngles)) {
    // Error: the joint angles we received force the motor out of range!
    // TODO(JS): Some kind of error sent over ROS
    return;
  }

  if (!actuateMotors(motorAngles)) {
    // Error: the motors failed to actuate to the desired angles!
    // TODO(JS): Some kind of error sent over ROS
    return;
  }

  // TODO(JS): Log success message
}

ros::Subscriber<sensor_msgs::JointState> sub(ROS_TOPIC_NAME, motorCallback);

void setup() {
  // ROS initialization
  nh.initNode();
  nh.subscribe(sub);

  // Set all servo pins as output
  pinMode(PIN_BASE_YAW, OUTPUT);
  pinMode(PIN_BOT_4B, OUTPUT);
  pinMode(PIN_TOP_4B, OUTPUT);
  pinMode(PIN_CAM_PITCH, OUTPUT);
  pinMode(PIN_CAM_YAW, OUTPUT);
  pinMode(PIN_CAM_ROLL, OUTPUT);

  // Attach servo objects to pins
  baseYawServo.attach(PIN_BASE_YAW);
  bot4BServo.attach(PIN_BOT_4B);
  top4BServo.attach(PIN_TOP_4B);
  camPitchServo.attach(PIN_CAM_PITCH);
  camYawServo.attach(PIN_CAM_YAW);
  camRollServo.attach(PIN_CAM_ROLL);
}

void loop() {
  // Spin in place, waiting for messages
  nh.spinOnce();
  delay(1);
}

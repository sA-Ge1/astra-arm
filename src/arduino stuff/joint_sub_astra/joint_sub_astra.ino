#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150  // Min pulse length count
#define SERVOMAX  600  // Max pulse length count

// Zero angles for each joint
int zero_shoulder = 90;
int zero_upperarm = 85;
int zero_elbow = 92;
int zero_wrist = 90;
int zero_extra_wrist = 90;

// Flags to specify if we add (true) or subtract (false) from zero angles
bool add_to_shoulder = true;
bool add_to_upperarm = false;
bool add_to_elbow = false;
bool add_to_wrist = true;
bool add_to_extra_wrist = true;

// Store joint values as integers relative to zero angles
int shoulder_joint = 0;
int upperarm_joint = 0;
int elbow_joint = 0;
int wrist_joint = 0;
int extra_wrist_joint = 0;

void setup() {
  Serial.begin(115200);  // Communication with ROS 2 node
  Serial.println("ESP32 ready to receive joint data.");
  Wire.begin(21, 22);//sda 21
  pwm.begin();
  pwm.setPWMFreq(50);  // Set frequency to 50 Hz for SG90 servos
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Read until newline
    parseJointData(data);
    printJointValues();

    // Set servo positionsw after printing the joint values
    pwm.setPWM(0, 0, angleToPulse(clampAngle(shoulder_joint)));       // Servo for shoulder
    pwm.setPWM(1, 0, angleToPulse(clampAngle(upperarm_joint)));      // Servo for upper arm
    pwm.setPWM(2, 0, angleToPulse(clampAngle(elbow_joint)));          // Servo for elbow
    pwm.setPWM(3, 0, angleToPulse(clampAngle(wrist_joint)));          // Servo for wrist
    pwm.setPWM(4, 0, angleToPulse(clampAngle(extra_wrist_joint)));    // Servo for extra wrist
  }
}

// Function to parse and adjust joint data from input string
void parseJointData(String data) {
  int start = 0;
  while (start < data.length()) {
    int end = data.indexOf(';', start);
    if (end == -1) end = data.length();
    String jointData = data.substring(start, end);

    int separator = jointData.indexOf(':');
    if (separator > 0) {
      String jointName = jointData.substring(0, separator);
      int jointValue = round(jointData.substring(separator + 1).toFloat());

      if (jointName == "shoulder_joint") {
        shoulder_joint = add_to_shoulder ? (zero_shoulder + jointValue) : (zero_shoulder - jointValue);
      } else if (jointName == "upperarm_joint") {
        upperarm_joint = add_to_upperarm ? (zero_upperarm + jointValue) : (zero_upperarm - jointValue);
      } else if (jointName == "elbow_joint") {
        elbow_joint = add_to_elbow ? (zero_elbow + jointValue) : (zero_elbow - jointValue);
      } else if (jointName == "wrist_joint") {
        wrist_joint = add_to_wrist ? (zero_wrist + jointValue) : (zero_wrist - jointValue);
      } else if (jointName == "extra_wrist_joint") {
        extra_wrist_joint = add_to_extra_wrist ? (zero_extra_wrist + jointValue) : (zero_extra_wrist - jointValue);
      }
    }

    start = end + 1;
  }
}

// Function to print current joint values
void printJointValues() {
  Serial.println("Adjusted joint values:");
  Serial.print("  Shoulder: "); Serial.println(shoulder_joint);
  Serial.print("  Upper Arm: "); Serial.println(upperarm_joint);
  Serial.print("  Elbow: "); Serial.println(elbow_joint);
  Serial.print("  Wrist: "); Serial.println(wrist_joint);
  Serial.print("  Extra Wrist: "); Serial.println(extra_wrist_joint);
}

// Helper function to map angle to PCA9685 pulse
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Helper function to clamp angle to 0-180 range
int clampAngle(int angle) {
  return constrain(angle, 0, 180);
}

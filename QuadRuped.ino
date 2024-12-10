#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "inverseKinematics.h"
#include "BluetoothSerial.h"

// Create a BluetoothSerial instance
BluetoothSerial SerialBT;
const int buzzerPin = 25;  

Adafruit_PWMServoDriver Servo = Adafruit_PWMServoDriver();

// const uint9_t Motors[4][3] = {{0, 1, 2},{3, 4, 5}, {6, 7, 8}, {9, 10, 11}};
const float a = 3, b = 13, c = 12.5; // needs to be meaxsured and changed
const float defaultAngles[12] = {0, 45, 90, 0, 135, -90, 0, 135, -90, 0, 45, 90};

uint16_t servoMin[12] = {100, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110};
uint16_t servoMax[12] = {600, 590, 590, 590, 590, 590, 590, 590, 590, 590, 590, 590};

uint16_t pulseLen(float angle, int motor){
  return (uint16_t)(servoMin[motor] + (angle*(servoMax[motor] - servoMin[motor])/180));  
}

float adjustAngle (float angle, int motor){
  if (motor%3 == 0){
    if (motor == 0 || motor == 9) return angle;
    else return 180 - angle;
  }

  if (motor%3 == 1) return 180 - angle;

  if (motor%3 == 2) {
    if (motor == 2 || motor == 11) return 180 - angle;
    else return -1*angle;
  }
} 

void moveServo(float angle, int i){
  Servo.setPWM(i, 0, pulseLen(adjustAngle(angle, i), i));
}

void restoreDefaultAngles() {
  for (int i = 0; i < 12; i++) moveServo(defaultAngles[i], i);
  delay(1000);
} 

void moveLeg (float angles[], int leg) {
  moveServo(angles[0], leg*3);

  if (leg == 0 || leg == 3) {
    moveServo(angles[1], leg*3 + 1);
    moveServo(angles[2], leg*3 + 2);
  } 
  else {
    moveServo(angles[3], leg*3 + 1);  
    moveServo(angles[4], leg*3 + 2);
  }
} 



void moveForward() {
  Serial.println("moveForward start");
  float center1 = 0, center2 = -18, radius = 3;
  float targets[3], angles[5];
  float commands[4][5];
  
  
  ////
  //the distance from 2d plane where our L1 and L2 are in motion
  targets[2] = a; // L1 remains constant
  ////

  unsigned long int phaseDuration = 1000; // Time for each phase in milliseconds
  unsigned long int startTime;
  float theta, l;

  // Phase 1: Legs 0 and 2 (circular), Legs 1 and 3 (linear)
  startTime = millis();
  while (millis() - startTime < phaseDuration) {
    float elapsedTime = millis() - startTime;
    float progress = elapsedTime / float(phaseDuration); // Progress [0, 1]
    
    // Circular motion for legs 0 and 2
    theta = 180 * progress; // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 + 1 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);

    commands[0][0] = angles[0];
    commands[0][1] = angles[1];
    commands[0][2] = angles[2];
    commands[0][3] = angles[3];
    commands[0][4] = angles[4];
    
    theta = 180 * (1-progress); // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2-1 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);

    commands[2][0] = angles[0];
    commands[2][1] = angles[1];
    commands[2][2] = angles[2];
    commands[2][3] = angles[3];
    commands[2][4] = angles[4];

    // Linear motion for legs 1 and 3
    l = center1 - radius + (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2-1;
    inverseKinematics(targets, angles);

    commands[3][0] = angles[0];
    commands[3][1] = angles[1];
    commands[3][2] = angles[2];
    commands[3][3] = angles[3];
    commands[3][4] = angles[4];

    l = center1 + radius - (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2+1;
    inverseKinematics(targets, angles);
    
    commands[1][0] = angles[0];
    commands[1][1] = angles[1];
    commands[1][2] = angles[2];
    commands[1][3] = angles[3];
    commands[1][4] = angles[4];

    // Move all servos
    /////
    //commands is a 4*5 array we need to give to moveServo function
    for(int leg=0;leg<=3;leg++){
      moveLeg(commands[leg],leg);
      delay(200);
    }
    /////////
     // Small delay for smoothness
  }

  // Phase 2: Legs 1 and 3 (circular), Legs 0 and 2 (linear)
  startTime = millis();
  while (millis() - startTime < phaseDuration) {
    float elapsedTime = millis() - startTime;
    float progress = elapsedTime / float(phaseDuration); // Progress [0, 1]
    
    // Circular motion for legs 1 and 3
    theta = 180 * progress; // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 - 1 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);

    commands[3][0] = angles[0];
    commands[3][1] = angles[1];
    commands[3][2] = angles[2];
    commands[3][3] = angles[3];
    commands[3][4] = angles[4];

    theta = 180 * (1-progress); // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 + 1 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);

    commands[1][0] = angles[0];
    commands[1][1] = angles[1];
    commands[1][2] = angles[2];
    commands[1][3] = angles[3];
    commands[1][4] = angles[4];

    // Linear motion for legs 0 and 2
    l = center1 - radius + (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2+1;
    inverseKinematics(targets, angles);

    commands[0][0] = angles[0];
    commands[0][1] = angles[1];
    commands[0][2] = angles[2];
    commands[0][3] = angles[3];
    commands[0][4] = angles[4];

    l = center1 + radius - (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2-1;
    inverseKinematics(targets, angles);

    commands[2][0] = angles[0];
    commands[2][1] = angles[1];
    commands[2][2] = angles[2];
    commands[2][3] = angles[3];
    commands[2][4] = angles[4];

    // Move all servos
    /////
    //commands is a 4*5 array we need to give to moveServo function
    for(int leg=0;leg<=3;leg++){
      moveLeg(commands[leg],leg);
      delay(200);
    }
    /////////
     // Small delay for smoothness
    
  }
  Serial.println("moveForward finish");
}

void moveBackward() {
  Serial.println("MoveBackward start");
  float center1 = 0, center2 = -18, radius = 3;
  float targets[3], angles[5];
  float commands[4][5];
  
  
  ////
  //the distance from 2d plane where our L1 and L2 are in motion
  targets[2] = a; // Height remains constant
  ////

  unsigned long phaseDuration = 1000; // Time for each phase in milliseconds
  unsigned long startTime;
  float theta, l;

  // Phase 1: Legs 0 and 2 (circular), Legs 1 and 3 (linear)
  startTime = millis();
  while (millis() - startTime < phaseDuration) {
    float elapsedTime = millis() - startTime;
    float progress = elapsedTime / float(phaseDuration); // Progress [0, 1]
    
    // Circular motion for legs 0 and 2
    theta = 180 * (1-progress); // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);
    
    commands[0][0] = angles[0];
    commands[0][1] = angles[1];
    commands[0][2] = angles[2];
    commands[0][3] = angles[3];
    commands[0][4] = angles[4];
    
    theta = 180 * progress; // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);
    
    commands[2][0] = angles[0];
    commands[2][1] = angles[1];
    commands[2][2] = angles[2];
    commands[2][3] = angles[3];
    commands[2][4] = angles[4];
    // Linear motion for legs 1 and 3
    l = center1 + radius - (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2;
    inverseKinematics(targets, angles);
    
    commands[3][0] = angles[0];
    commands[3][1] = angles[1];
    commands[3][2] = angles[2];
    commands[3][3] = angles[3];
    commands[3][4] = angles[4];

    l = center1 - radius + (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2;
    inverseKinematics(targets, angles);
    
    commands[1][0] = angles[0];
    commands[1][1] = angles[1];
    commands[1][2] = angles[2];
    commands[1][3] = angles[3];
    commands[1][4] = angles[4];

    // Move all servos
    /////
    //commands is a 4*5 array we need to give to moveServo function
    for(int leg=0;leg<=3;leg++){
      moveLeg(commands[leg],leg);
    }
    /////////
    //delay(20); // Small delay for smoothness
  }

  // Phase 2: Legs 1 and 3 (circular), Legs 0 and 2 (linear)
  startTime = millis();
  while (millis() - startTime < phaseDuration) {
    float elapsedTime = millis() - startTime;
    float progress = elapsedTime / float(phaseDuration); // Progress [0, 1]
    
    // Circular motion for legs 1 and 3
    theta = 180 * (1-progress); // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);

    commands[3][0] = angles[0];
    commands[3][1] = angles[1];
    commands[3][2] = angles[2];
    commands[3][3] = angles[3];
    commands[3][4] = angles[4];

    theta = 180 * progress; // Interpolate theta from 0 to 180
    targets[0] = center1 + radius * cos(theta * PI / 180);
    targets[1] = center2 + radius * sin(theta * PI / 180);
    inverseKinematics(targets, angles);

    commands[1][0] = angles[0];
    commands[1][1] = angles[1];
    commands[1][2] = angles[2];
    commands[1][3] = angles[3];
    commands[1][4] = angles[4];

    // Linear motion for legs 0 and 2
    l = center1 + radius - (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2;
    inverseKinematics(targets, angles);

    commands[0][0] = angles[0];
    commands[0][1] = angles[1];
    commands[0][2] = angles[2];
    commands[0][3] = angles[3];
    commands[0][4] = angles[4];
    
    l = center1 - radius + (2 * radius * progress); // Interpolate l from center1 - radius to center1 + radius
    targets[0] = l;
    targets[1] = center2;
    inverseKinematics(targets, angles);

    commands[2][0] = angles[0];
    commands[2][1] = angles[1];
    commands[2][2] = angles[2];
    commands[2][3] = angles[3];
    commands[2][4] = angles[4];

    // Move all servos
    /////
    //commands is a 4*5 array we need to give to moveServo function
    for(int leg=0;leg<=3;leg++){
      moveLeg(commands[leg],leg);
    }
    /////////
    delay(20); // Small delay for smoothness
  }
  Serial.println("MoveBackward finish");
}



void generateBark() {
  // Simulate a "woof-woof" sequence with low pitch (for bulldog-like sound)
  for (int i = 0; i < 2; i++) {
    playTone(1200, 150);
    delay(150);
    playTone(500, 100);
    //delay(300);
  }
}

void playTone(int frequency, int duration) {
  // Set tone frequency for the buzzer
  ledcWriteTone(buzzerPin, frequency);

  delay(duration);
  
  // Turn off the buzzer (stop PWM signal)
  ledcWrite(buzzerPin, 0);
}



void setup() {
  Serial.begin(115200);
  Servo.begin();
  Servo.setPWMFreq(60);
  delay(10);
  pinMode(buzzerPin, OUTPUT);
  ledcAttach(buzzerPin, 1000, 8);

  // Start Bluetooth with a name (e.g., "ESP32_Bot")
  SerialBT.begin("Quadruped_Bot");
  Serial.println("Bluetooth started. Waiting for connections...");

  restoreDefaultAngles(); 
  delay(10000);


}

void loop() {
  float targets[3], angles[5];
  moveForward();
  // float targets[3], angles[5];
  // uint8_t commandArray[16]={128,128,128,128,0,0,0,0,0,0,0,0,0,0,0,0};
  // if (SerialBT.available()) {
  //    // Array to store incoming bytes
  //   SerialBT.readBytes(commandArray, sizeof(commandArray));
  //   Serial.println();

  //   // Print received data to Serial Monitor
    


  //   // Movement logic based on received data
  // }
  // Serial.print("Received data: ");
  // for (int i = 0; i < sizeof(commandArray); i++) {
  //   Serial.print(commandArray[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // if(commandArray[1]<20){
  //   moveForward();
  //   while (SerialBT.available()) {
  //       SerialBT.read(); // Discard any remaining bytes
  //     }
  // }
  // else if(commandArray[1]>235){
  //   moveBackward();
  //   while (SerialBT.available()) {
  //       SerialBT.read(); // Discard any remaining bytes
  //     }
  // }
  // if(commandArray[8]==1){
  //   generateBark();
  //   while (SerialBT.available()) {
  //       SerialBT.read(); // Discard any remaining bytes
  //     }
  // }

  // /*
  // for (float h = 15; h < 25; h = h + 0.05) {
  //   targets[0] = 0;
  //   targets[1] = -1*h;
  //   targets[2] = a;
  //   inverseKinematics(targets, angles);    
  //   for (int legg = 0; legg < 4; legg++) moveLeg(angles, legg);
  //   //delay(20);
  // }

  // delay(50);

  // for (float h = 25; h > 15; h = h - 0.05) {
  //   targets[0] = 0;
  //   targets[1] = -1*h;
  //   targets[2] = a;
  //   inverseKinematics(targets, angles);    
  //   for (int legg = 0; legg < 4; legg++) moveLeg(angles, legg);
  //   //delay(20);
  // }*/
}
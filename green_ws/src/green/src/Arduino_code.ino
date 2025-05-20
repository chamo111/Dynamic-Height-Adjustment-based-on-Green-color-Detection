#if (ARDUINO >= 100) 
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif


#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "Wire.h"
#include "Arduino.h"
#include "ArduinoHardware.h"

ros::NodeHandle nh;


const int motorPin1 = 3;  // Example motor control pins
const int motorPin2 = 4;
const int enablePin1 = 5;

//const int motorPin3 = 6;  // Example motor control pins
//const int motorPin4 = 7;
//const int enablePin2 = 8;

const float speed = 9.5;  // Speed of the actuator in mm/s
const int desiredLength = -20;  // Desired length to open in mm
float currentDistance1 = 0;
//float currentDistance2 = 0;
float previousDistance1 = 0;
//float previousDistance2 = 0;


void messageCallback(const std_msgs::Float32MultiArray& msg)
{
  //Print the received message
  //Serial1.print("We received: ");
  //Serial.println(msg.data);
  currentDistance1 = msg.data[0];
  //currentDistance2 = msg.data[1];
  if (currentDistance1 > previousDistance1){
    moveActuatorForward1(currentDistance1-previousDistance1);
    //moveActuatorForward2(currentDistance2-previousDistance2);
  }
  
  else{
    moveActuatorBackward1(previousDistance1-currentDistance1);
    //moveActuatorBackward2(previousDistance2-currentDistance2);
    }

    previousDistance1 = currentDistance1;  
    delay(10000);

    }
    
  
  
//  //seconddddddd
//  if (currentDistance2 > previousDistance2){
//    moveActuatorForward2(currentDistance2-previousDistance2);
//  }
//  else{
//    moveActuatorBackward2(previousDistance2-currentDistance2);
//    }
//  previousDistance2 = currentDistance2;  
//  delay(10000);
  
//}

std_msgs::Float32MultiArray msg;
ros::Subscriber<std_msgs::Float32MultiArray> sub("information", &messageCallback);

void setup() {
  Serial1.begin(115200);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  //pinMode(motorPin3, OUTPUT);
  //pinMode(motorPin4, OUTPUT);
  //pinMode(enablePin2, OUTPUT);

  
  nh.getHardware()->setBaud(57600); 
  nh.initNode();
  nh.subscribe(sub);
 

}



void loop() {

   nh.spinOnce();


}




void moveActuatorForward1(int length) {
  // Calculate time to move the specified length (in seconds)
  
      float timeToMove = length / speed;  // time in seconds
  
      // Convert time to milliseconds
      unsigned long timeToMoveMillis = timeToMove * 1000;

      // Activate the actuator (open it)
      digitalWrite(motorPin1, LOW);     // Set the direction (open)
      digitalWrite(motorPin2, HIGH);    
      analogWrite(enablePin1, 255);       // Full speed (255 is maximum for PWM)

      // Wait for the actuator to move the specified distance
      delay(timeToMoveMillis);

      // Stop the actuator after moving the desired distance
      digitalWrite(motorPin1, LOW);     // Set the direction (open)
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin1, 0);         // Stop the motor by turning off PWM
}


//void moveActuatorForward2(int length) {
  // Calculate time to move the specified length (in seconds)
  
      //float timeToMove = length / speed;  // time in seconds
  
      // Convert time to milliseconds
      //unsigned long timeToMoveMillis = timeToMove * 1000;

      // Activate the actuator (open it)
      //digitalWrite(motorPin3, LOW);     // Set the direction (open)
      //digitalWrite(motorPin4, HIGH);    
      //analogWrite(enablePin2, 255);       // Full speed (255 is maximum for PWM)

      // Wait for the actuator to move the specified distance
      //delay(timeToMoveMillis);

      // Stop the actuator after moving the desired distance
      //digitalWrite(motorPin3, LOW);     // Set the direction (open)
      //digitalWrite(motorPin4, LOW);
      //analogWrite(enablePin2, 0);         // Stop the motor by turning off PWM
//}


void moveActuatorBackward1(int length) {

      float timeToMove = length / speed;  // time in seconds
  
      // Convert time to milliseconds
      unsigned long timeToMoveMillis = timeToMove * 1000;

      // Activate the actuator (open it)
      digitalWrite(motorPin1, HIGH);     // Set the direction (open)
      digitalWrite(motorPin2, LOW);    
      analogWrite(enablePin1, 255);       // Full speed (255 is maximum for PWM)

      // Wait for the actuator to move the specified distance
      delay(timeToMoveMillis);

      // Stop the actuator after moving the desired distance
      digitalWrite(motorPin1, LOW);     // Set the direction (open)
      digitalWrite(motorPin2, LOW);    
      analogWrite(enablePin1, 0);         // Stop the motor by turning off PWM
    
   
    
    
    }


//void moveActuatorBackward2(int length) {

      //float timeToMove = length / speed;  // time in seconds
  
      // Convert time to milliseconds
      //unsigned long timeToMoveMillis = timeToMove * 1000;

      // Activate the actuator (open it)
      //digitalWrite(motorPin3, HIGH);     // Set the direction (open)
      //digitalWrite(motorPin4, LOW);    
      //analogWrite(enablePin2, 255);       // Full speed (255 is maximum for PWM)

      // Wait for the actuator to move the specified distance
      //delay(timeToMoveMillis);

      // Stop the actuator after moving the desired distance
      //digitalWrite(motorPin3, LOW);     // Set the direction (open)
      //digitalWrite(motorPin4, LOW);    
      //analogWrite(enablePin2, 0);         // Stop the motor by turning off PWM
    
    
    
    
    //}


  

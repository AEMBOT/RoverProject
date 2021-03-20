//Arduino Program that handles all external interfaces over I2C

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <Wire.h>

#include "RoboClaw.h"

// All of the motors controlling the 
Servo front_left_servo;
Servo front_right_servo;
Servo back_left_servo;
Servo back_right_servo;

// Array to hold all the servos and make them easier to use
Servo servo_array[4];

// Set I2C Device address
#define SLAVE_ADDRESS 0x08

// Represents what data we are recieving
int register_id = 0;

// Array to hold speeds for each of the 6 motors, default them all to stopped
int motor_speeds[] = {64, 64, 64, 64, 64, 64};

// Values from the last loop to not send tons of information to the controller, only when new info is needed
int last_motor_speeds[] = {64, 64, 64, 64, 64, 64};

// Array to hold the s
int wheel_angle[] = {0, 0, 0, 0};

// Array to hold the previous angles of each of the servos
int last_wheel_angles[] = {0, 0, 0, 0};


//Set the serial to run on the standard TX and RX pins
SoftwareSerial serial(0,1);	
RoboClaw roboclaw(&serial,10000);

// Address of the first driving motor controller
#define drive_controller_addr_1 0x80
#define drive_controller_addr_2 0x81
#define drive_controller_addr_3 0x82

// Address of the first steering motor controller.
#define steering_controller_addr_1 0x83
#define steering_controller_addr_2 0x84

void setup() {

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);

  //Open roboclaw serial ports
  roboclaw.begin(115200);
}

/**
 * When data is received by the slave arduino
 */ 
void receiveData(int bytecount)
{

  // Check the first byte to see where the information we are receiving should be stored
  register_id = int(Wire.read());

  // If the information is meant to be sent to the drive train motor controllers send it there
  if (register_id == 0){
    // Set the new motor speeds
    for (int i = 0; i < bytecount; i++) {
      motor_speeds[i] = int(Wire.read());
    }
  }

  // Get information intended for the servos and store them in the servo array
  else if (register_id == 1){
    for (int i = 0; i < bytecount; i++){
      wheel_angle[i] = gearedToEncoder(int(Wire.read()));
    }
  }
}
/**
 * Array to Controller/Motor Mappings
 * 0 - Drive Train Motor ID
   * 0 - Controller 1 : Motor 1
   * 1 - Controller 1 : Motor 2
   * 2 - Controller 2 : Motor 1
   * 3 - Controller 2 : Motor 2
   * 4 - Controller 3 : Motor 1
   * 5 - Controller 3 : Motor 2
 * 1 - Steering Control
   * 0 - Front Left 
   * 1 - Front Right 
   * 2 - Back Left 
   * 3 - Back Right 
 */

void loop() {
    
  // Using sizeof the array divided by the sizeof an element in the array to get the length of the array of motors
  for(int i=0; i<(sizeof(motor_speeds) / sizeof(motor_speeds[0])); i++){
      if (motor_speeds[i] != last_motor_speeds[i]){
        switch (i)
        {
          case 0:
            roboclaw.ForwardBackwardM1(drive_controller_addr_1,motor_speeds[i]);
            break;
          case 1:
            roboclaw.ForwardBackwardM2(drive_controller_addr_1,motor_speeds[i]); 
            break;
          case 2:
            roboclaw.ForwardBackwardM1(drive_controller_addr_2,motor_speeds[i]);
            break;
          case 3:
            roboclaw.ForwardBackwardM2(drive_controller_addr_2,motor_speeds[i]);
            break;
          case 4:
            roboclaw.ForwardBackwardM1(drive_controller_addr_3,motor_speeds[i]);
            break;
          case 5:
            roboclaw.ForwardBackwardM2(drive_controller_addr_3,motor_speeds[i]); 
            break;
        }

        // Set the last to the current
        last_motor_speeds[i] = motor_speeds[i];
      }
  }

  // Get all values in the wheel_angles array
  for (int i=0; i<(sizeof(wheel_angles) / sizeof(wheel_angles[0])); i++){

    // Check if the new angle is the same as last to avoid sending useless information
    if (wheel_angles[i] != last_wheel_angles[i]){
      switch (i)
        {
          case 0:
            roboclaw.ForwardBackwardM1(steering_controller_addr_1, getWheelPowers(wheel_angle[i]));
            break;
          case 1:
            roboclaw.ForwardBackwardM2(steering_controller_addr_1, getWheelPowers(wheel_angle[i])); 
            break;
          case 2:
            roboclaw.ForwardBackwardM1(steering_controller_addr_2, getWheelPowers(wheel_angle[i]));
            break;
          case 3:
            roboclaw.ForwardBackwardM2(steering_controller_addr_2, getWheelPowers(wheel_angle[i]));
            break;
        }

      // Set the last to the current
      last_wheel_angles[i] = wheel_angles[i];
    }
  }
}

// Convert the angle we want the wheels to be facing into a motor power that PID is applied to, to reach the goal
int getWheelPowers(int currentAngle){

  // TODO: PID It up
  return input; 
}
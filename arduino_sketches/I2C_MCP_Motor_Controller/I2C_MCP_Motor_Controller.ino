//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <Servo.h>
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

// Array to hold the angles of each of the servos
int wheel_angles[] = {0, 0, 0, 0};

// Array to hold the previous angles of each of the servos
int last_wheel_angles[] = {0, 0, 0, 0};

//Set the serial to run on the standard TX and RX pins
SoftwareSerial serial(0,1);	
RoboClaw roboclaw(&serial,10000);

// Address of the first motor controller
#define controller_1_addr 0x80

void setup() {

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);

  //Open roboclaw serial ports
  roboclaw.begin(115200);

  // Run the front left side servo on pin 9
  front_left_servo.attach(9);

  // Run the front right side servo on pin 10
  front_right_servo.attach(10)

  // Run the back left side servo on pin 5
  back_left_servo.attach(5);

  // Run the back right side servo on pin 6
  back_right_servo.attach(6);

  // Add all servos to spots in the servo array
  servo_array[0] = front_left_servo;
  servo_array[1] = front_right_servo;
  servo_array[2] = back_left_servo;
  servo_array[3] = back_right_servo;

}

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
      wheel_angles[i] = angle_to_servo(int(Wire.read()));
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
 * 1 - Servo Turning Control
   * 0 - Front Left Servo
   * 1 - Front Right Servo
   * 2 - Back Left Servo
   * 3 - Back Right Servo
 */

void loop() {
    
    // Using sizeof the array divided by the sizeof an element in the array to get the length of the array of motors
    for(int i=0; i<(sizeof(motor_speeds) / sizeof(motor_speeds[0])); i++){
       if (motor_speeds[i] != last_motor_speeds[i]){

          // Drive the first motor on the first controller
          if (i == 0){
            roboclaw.ForwardBackwardM1(controller_1_addr,motor_speeds[i]);
          }
          
          // Drive the second motor on the first controller
          else if (i == 1){
            roboclaw.ForwardBackwardM2(controller_1_addr,motor_speeds[i]); 
          }
           
  
          // Set the last to the current
          last_motor_speeds[i] = motor_speeds[i];
       }
    }

    // Get all values in the wheel_angles array
    for (int i=0; i<(sizeof(wheel_angles) / sizeof(wheel_angles[0])); i++){

      // Check if the new angle is the same as last to avoid sending useless information
      if (wheel_angles[i] != last_wheel_angles[i]){
        // Write the angle to the corresponding servo 
        servo_array[i].write(wheel_angles[i])

        // Set the last to the current
        last_wheel_angles[i] = wheel_angles[i];
      }
    }
    
}

// Converts normal angles into matching angles of the geared down 
int angle_to_servo(int input){

  // Implement at a later date
  return input
}
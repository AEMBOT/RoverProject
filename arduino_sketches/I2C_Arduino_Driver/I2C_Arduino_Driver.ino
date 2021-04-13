//Arduino Program that handles all external interfaces over I2C

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <Wire.h>

#include "RoboClaw.h"

// Set I2C Device address
#define SLAVE_ADDRESS 0x08

// Represents what data we are recieving
int register_id = 0;

// Array to hold on inputs provided to the motors 
int motor_inputs[] = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64};

// Values from the last loop to not send tons of information to the controller, only when new info is needed
int last_motor_inputs[] = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64};

//Set the serial to run on PWM pins 10 and 11 with the hopes that we can regain basic USB serial interaction
SoftwareSerial serial(10,11);	
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
  int receivedByte = 0;

  // Counter gets reset after a new reg-id arrives
  int i = 0;


  // Use wire.availale to read out all recieved data
  while (Wire.available()){

    // Get and stor the byte so we can access it in multiple places
    receivedByte = int(Wire.read());

    // Check if the byte number was high enough to be a register byte
    if (receivedByte == 222){
      register_id = receivedByte;

      // Reset the array counter
      i = 0;
    }

    // After checking if our last byte was a reg-id check which reg ID it was and if it was the driving one act accordingly
    if (register_id == 222){
      motor_inputs[i] = receivedByte;
      i++;
    }
  }
}
/**
 * Array to Controller/Motor Mappings
 * Drive Train Motor ID
   * 0 - Controller 1 : Motor 1
   * 1 - Controller 1 : Motor 2
   * 2 - Controller 2 : Motor 1
   * 3 - Controller 2 : Motor 2
   * 4 - Controller 3 : Motor 1
   * 5 - Controller 3 : Motor 2
 * Steering Control
   * 6 - Controller 4 : Front Left 
   * 7 - Controller 4 : Front Right 
   * 8 - Controller 5 : Back Left 
   * 9 - Controller 5 : Back Right 
 */

void loop() {
    
  // Using sizeof the array divided by the sizeof an element in the array to get the length of the array of motors
  for(int i=0; i<(sizeof(motor_inputs) / sizeof(motor_inputs[0])); i++){
      if (motor_inputs[i] != last_motor_inputs[i]){
        switch (i)
        {

          // Drive
          case 0:
            roboclaw.ForwardBackwardM1(drive_controller_addr_1,motor_inputs[i]);
            break;
          case 1:
            roboclaw.ForwardBackwardM2(drive_controller_addr_1,motor_inputs[i]); 
            break;
          case 2:
            roboclaw.ForwardBackwardM1(drive_controller_addr_2,motor_inputs[i]);
            break;
          case 3:
            roboclaw.ForwardBackwardM2(drive_controller_addr_2,motor_inputs[i]);
            break;
          case 4:
            roboclaw.ForwardBackwardM1(drive_controller_addr_3,motor_inputs[i]);
            break;
          case 5:
            roboclaw.ForwardBackwardM2(drive_controller_addr_3,motor_inputs[i]); 
            break;

          // Steering 
          case 6:
            roboclaw.ForwardBackwardM1(steering_controller_addr_1, getWheelPowers(motor_inputs[i]));
            break;
          case 7:
            roboclaw.ForwardBackwardM2(steering_controller_addr_1, getWheelPowers(motor_inputs[i]));
            break;
          case 8:
            roboclaw.ForwardBackwardM1(steering_controller_addr_2, getWheelPowers(motor_inputs[i]));
            break;
          case 9:
            roboclaw.ForwardBackwardM2(steering_controller_addr_2, getWheelPowers(motor_inputs[i]));
          
        }

        // Set the last to the current
        last_motor_inputs[i] = motor_inputs[i];
      }
  }
}

// Convert the angle we want the wheels to be facing into a motor power that PID is applied to, to reach the goal
int getWheelPowers(int input){

  // TODO: PID It up
  return input; 
}

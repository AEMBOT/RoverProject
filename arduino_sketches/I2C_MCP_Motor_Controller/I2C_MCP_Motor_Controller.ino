//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RoboClaw.h"

// Set I2C slave device address
#define SLAVE_ADDRESS 0x08

// Array to hold speeds for each of the 6 motors, default them all to stopped
int motor_speeds[] = {64, 64, 64, 64, 64, 64, 64};

// Values from the last loop to not send tons of information to the controller, only when new info is needed
int last_motor_speeds[] = {64, 64, 64, 64, 64, 64, 64};

//Adds the ability to set the serial to run on differnet ports, currently set to standard TX and RX ports
SoftwareSerial serial(0,1);	

// Create a new motor controller controller, used to command all controllers on the bus
RoboClaw roboclaw(&serial, 10000);

// Set the address for the first motor controller, 0x80 = 128
#define controller_1_addr 0x80

void setup() {

  // Broadcast the arduino as slave on the I2C network at the set address
  Wire.begin(SLAVE_ADDRESS);

  // Set function to control what happens to received data
  Wire.onReceive(receiveData);

  //Begin waiting for commands to be sent to the controllers with a baudrate of 115200
  roboclaw.begin(115200);
}

/**
 * When new data is received on the I2C bus
 */
void receiveData(int bytecount)
{

  //For each byte received read the byte and set it to an index within the motor_speeds array
  for (int i = 0; i < bytecount; i++) {
    motor_speeds[i] = int(Wire.read());
  }
  
}
/**
 * Array to Controller/Motor Mappings
 * 0 - Null Byte / Starting Byte
 * 1 - Controller 1 : Motor 1
 * 2 - Controller 1 : Motor 2
 * 3 - Controller 2 : Motor 1
 * 4 - Controller 2 : Motor 2
 * 5 - Controller 3 : Motor 1
 * 6 - Controller 3 : Motor 2
 */

void loop() {

  /**
   *  Loop through every value inside the motor_speeds array execpt for the first one which is a null byte. 
   *  Note: sizeof(motor_speeds) returns the size in bytes which you then need to divide by the size of the values to get the total length of the array
   */
  for(int i=1; i<(sizeof(motor_speeds) / sizeof(motor_speeds[0])); i++){

    // Check to see if the current motor speed is not equal to the last as to not send too much info to the controller
    if (motor_speeds[i] != last_motor_speeds[i]){

      // Drive the first motor on the first controller
      if (i == 1){
        roboclaw.ForwardBackwardM1(controller_1_addr,motor_speeds[i]);
      }
          
      // Drive the second motor on the first controller
      else if (i == 2){
          roboclaw.ForwardBackwardM2(controller_1_addr,motor_speeds[i]); 
      }

      // Add more motor indices as they are needed      
  
      // Set the last to the current
      last_motor_speeds[i] = motor_speeds[i];
    }
  }
}
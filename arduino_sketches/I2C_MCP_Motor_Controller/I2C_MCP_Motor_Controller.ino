//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RoboClaw.h"

// Set I2C Device address
#define SLAVE_ADDRESS 0x08

// Represents what data we are recieving
int register_id = 0;

// Array to hold speeds for each of the 6 motors, default them all to stopped
int motor_speeds[] = {64, 64, 64, 64, 64, 64};

// Values from the last loop to not send tons of information to the controller, only when new info is needed
int last_motor_speeds[] = {64, 64, 64, 64, 64, 64};

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(0,1);	
RoboClaw roboclaw(&serial,10000);

#define controller_1_addr 0x80

void setup() {

   Wire.begin(SLAVE_ADDRESS);
   Wire.onReceive(receiveData);
  //Open roboclaw serial ports
  roboclaw.begin(115200);
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
 */

void loop() {
    
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
    
}
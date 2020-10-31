#include <Wire.h>

// The arbitrary device address to communicate over
#define DEVICE_ADDRESS 0x05

// Data response formated as a string so it can easily store both integers and doubles
String response_data = "";

void setup()
{

  // Listen as a follower on the specified address
  Wire.begin(DEVICE_ADDRESS);

  // Configure response call backs for the I2C interface
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop(){ /* Loop forever with a 200ms delay*/ delay(200); }

/**
 * Called when data is received from the master
 * 
 * byteCont: Number of bytes received
 */
void receiveData(int bytyeCount)
{

  String commandStr = "";
  // Loop through all avialiable bytes
  while (Wire.available())
  {

    // Append the character read into a full command
    commandStr += Wire.read();
  }

  // Response from the parsing and execution of the command
  response_data = parseCommand(commandStr);
}

/**
 * Called when data needs to be returned to the master
 */
void sendData()
{
  
  // Create a char array with the same size as the response data and copy the response data into that array
  char response[response_data.length()];
  response_data.toCharArray(response, response_data.length());

  // Send the char array response over the I2C bus and then reset the response data
  Wire.write(response);
  response_data = "";
}

/**
 * Parse and execute the string constructed from the I2C Response
 * 
 * return: The value to return, returns -1 if no data was returned
 */
double parseCommand(String commandStr)
{

  // Split the command string from one past the first space to the end of the string and convert the string to an int
  int pinNumber = commandStr.substring(commandStr.indexOf(' ') + 1, commandStr.length() - 1).toInt();

  // Get the fist two characters from the command string that represent the command type
  String commandType = commandStr.substring(0, 1);

  // Read the data off a digital pin
  if (commandType.equals("dr"))
  {

    // Read the data from a digital pin
    pinMode(pinNumber, INPUT);
    return (int) digitalRead(pinNumber);
  }

  else if (commandType.equals("ar"))
  {

    // Read the data from an analog pin
    return analogRead(pinNumber);
  }

  else if (commandType.equals("dw"))
  {
    // Needs interface re-design, not implementing yet as we may not need it
  }
}

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
 * byteCount: Number of bytes received
 */
void receiveData(int byteCount)
{

  String commandStr = "";
  // Loop through all available bytes
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
String parseCommand(String commandStr)
{
  // Pin Number to read / write data to
  int pinNumber = 0;
  
  
  // Check to see if there is another space after the first one
  if (commandStr.lastIndexOf(' ') < 3){

    // If there is no space something is wrong, return
    if(commandStr.indexOf(' ') == -1){
      return "Invalid command structure";
    }
    
    int firstDelimIndex = commandStr.indexOf(' ');

    // Split the command string from one past the first space to the end of the string and convert the string to an int
    pinNumber = commandStr.substring(firstDelimIndex + 1, commandStr.length() - 1).toInt();
  }

  // If there is split differently to only get the pin
  else{
    if(commandStr.indexOf(' ') == -1){
      return "Invalid command structure";
    }
    
    int firstDelimIndex = commandStr.indexOf(' ');
    
    // Get only the middle number by substringing from the first space to the second space
    pinNumber = commandStr.substring(firstDelimIndex + 1, commandStr.indexOf(' ', firstDelimIndex + 1) - 1).toInt();
  }
  
  // Get the fist two characters from the command string that represent the command type
  String commandType = commandStr.substring(0, 1);

  // Read the data off a digital pin
  if (commandType.equals("dr"))
  {

    // Read the data from a digital pin
    pinMode(pinNumber, INPUT);
    return String(digitalRead(pinNumber));
  }

  else if (commandType.equals("ar"))
  {
    // Read the data from an analog pin
    return String(analogRead(pinNumber));
  }

  // If we are trying to preform a digital write
  else if (commandType.equals("dw"))
  {
    // Get one index past the second space to the end of the string
    int value = commandStr.substring(commandStr.indexOf(' ', commandStr.indexOf(' ') + 1) + 1, commandStr.length() - 1).toInt();

    // Set the pin mode to output so we can write to it
    pinMode(pinNumber, OUTPUT);

    // If a LOW write is sent
    if (value == 0){
      digitalWrite(pinNumber, LOW);
    }
    else if(value == 1){
      digitalWrite(pinNumber, HIGH);
    }
    else{
      return "invalid write value";
    }
    return "ok";
  }

  else if (commandType.equals("aw")){

    // Get one index past the second space to the end of the string to get the input value
    int value = commandStr.substring(commandStr.indexOf(' ', commandStr.indexOf(' ') + 1) + 1, commandStr.length() - 1).toInt();

    // Set the pin mode to output so we can write to it
    pinMode(pinNumber, OUTPUT);

    // Write the PWM value to a pin
    analogWrite(pinNumber, value)

    return "ok";
  }
  // If none of those were triggered the command didnt execute
  else{
    return "Invalid command";
  }
}

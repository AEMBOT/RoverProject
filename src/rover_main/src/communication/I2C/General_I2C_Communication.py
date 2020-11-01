#!/usr/bin/python3

import smbus2

from unittest.mock import Mock

from rover_main.srv import i2c_commsResponse

class I2C:

    @staticmethod
    def send_data(addr: int, data: str):
        """Send and recieve data from the I2C Device"""

        # Connect to the correct I2C Bus
        bus = smbus2.SMBus(0)
      

        # Convert the given string into a byte array
        byte_message = I2C.string_to_bytes(data)

        # Write the byte array to the I2C bus
        bus.write_i2c_block_data(i2c_addr=addr,
                                register=0x00,
                                data=byte_message)

        # Read the response bytes
        response_recieved = bus.read_i2c_block_data(i2c_addr=addr,
                                                    register=0x00,
                                                    length=15)

        # Convert those bytes back into a string and submit that as the service response
        response_string = I2C.bytes_to_string(response_recieved)
        return i2c_commsResponse(response_string)


    @staticmethod
    def string_to_bytes(input: str):
        """Convert a given string to a byte list"""
        retVal = []
        for c in input:
                retVal.append(ord(c))
        return retVal
    
    @staticmethod
    def bytes_to_string(input: list):
        """Convert a given byte list to a string"""
        retVal = ""
        for c in input:
                retVal += chr(c)
        return retVal
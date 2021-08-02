import math

"""Used to handle math for controlling the wheel differential"""

# Distance betwee the 2 sets of tires (meters)
TRACK_WIDTH = 1

# Distance between the front axel and back axel (meters)
AXLE_DISTANCE = 1

# Current radii of the circle to calculate angles from
circleRadius = 0

# Whether or not to reorder values to turn the other direction
turnRight = False


def updateCircleRadius(turnRadius, controllerInputs):
    global circleRadius, turnRight
    """Updates the inner circles radius"""
    b = 0.52345 / turnRadius
    circleRadius = ((0.5 * AXLE_DISTANCE) / math.sin(b))
    turnRight = controllerInputs > 0

def _getInnerCircleY(x):
    """Get the Y axis value at a given X of the inner circle"""
    return math.sqrt(math.pow(circleRadius, 2) - math.pow(x, 2))

def _getInnerCircleX(y):
    """Get the X axis value at a give Y of the inner circle"""
    return math.sqrt(-math.pow(y, 2) + math.pow(circleRadius, 2))

def _getOuterCircleY(x):
    """Get the Y axis value at a given X of the outer circle"""
    return math.sqrt(math.pow(circleRadius + TRACK_WIDTH, 2) - math.pow(x, 2))

def _getOuterCircleX(y):
    """Get the X axis value at a give Y of the inner circle"""
    return math.sqrt(-math.pow(y, 2) + math.pow(circleRadius + TRACK_WIDTH, 2))

def _getInnerCircleDerivative(x):
    """Get the derivative at a given X on the inner circle"""
    return -(x / _getInnerCircleY(x))

def _getOuterCircleDerivative(x):
    """Get the derivative at a given X on the outer circle"""
    return -(x / _getOuterCircleY(x))

def _convertSlopeToAngle(x):
    """Convert a given slope into an angle"""
    return math.degrees(abs(math.atan(x)))

def remapBounds(low1, low2, high1, high2, value):
    """Remap the given value into a new range"""
    return (low2 + (value - low1) * (high2 - low2) / (high1 - low1))

def _calcSpeedIncrease(value):
    """Calculate the speed increase for the given wheel"""
    innerCircumfrence = 2 * math.pi * circleRadius
    outerCircumfrence = 2 * math.pi * (circleRadius+TRACK_WIDTH)

    return (innerCircumfrence / outerCircumfrence) * value

def map_to_motor_controller(value):
    """Convert from -1 to 1 to 0 to 127 with 64 in the middle to accommodate the motor controller values"""
    if value >= 0:
        if 64 + abs(round(value * 64)) > 127:
            return 127
        return (64 + abs(round(value * 64)))
    else:
        return (64 - abs(round(value * 64)))

def getNormalizedWheelSpeed(value, angle_input, wheel):
    """Prevent the wheel speeds from exceeding 1 to allow for it to still turn while at full power"""

    speedDivisor = 1
    outerSpeed = value + _calcSpeedIncrease(value)
    if outerSpeed > 1:
        speedDivisor = outerSpeed

    # If we are trying to turn
    if angle_input != 0:
        if not turnRight:
            # Front Left
            if wheel == 0:
                return map_to_motor_controller(value / speedDivisor)

            # Front Right
            elif wheel == 1:
                return map_to_motor_controller(outerSpeed / speedDivisor)
            
            # Back Left
            elif wheel == 2:
                return map_to_motor_controller(value / speedDivisor)

            # Back Right
            elif wheel == 3:
                return map_to_motor_controller(outerSpeed / speedDivisor)

            # Middle Right
            elif wheel == 4:
                return map_to_motor_controller(outerSpeed / speedDivisor)
            # Middle Left
            elif wheel == 5:
                return map_to_motor_controller(value / speedDivisor)
                
        else:
            # Front Left
            if wheel == 0:
                return map_to_motor_controller(outerSpeed / speedDivisor)

            # Front Right
            elif wheel == 1:
                return map_to_motor_controller(value / speedDivisor)
            
            # Back Left
            elif wheel == 2:
                return map_to_motor_controller(outerSpeed / speedDivisor)

            # Back Right
            elif wheel == 3:
                return map_to_motor_controller(value / speedDivisor)

            # Middle Right
            elif wheel == 4:
                return map_to_motor_controller(value / speedDivisor)
            # Middle Left
            elif wheel == 5:
                return map_to_motor_controller(outerSpeed / speedDivisor)
    else:
        return map_to_motor_controller(value)

def getWheelAngle(wheel, angleInput):
    """Returns the required angle of the wheels enum"""
    # Turning Left
    if angleInput != 0:
        if not turnRight:
            if wheel == 0:
                return _convertSlopeToAngle(_getInnerCircleDerivative(_getInnerCircleX(AXLE_DISTANCE / 2))) + 90
            elif wheel == 1:
                return _convertSlopeToAngle(_getOuterCircleDerivative(_getOuterCircleX(AXLE_DISTANCE / 2))) + 90
            elif wheel == 2:
                return 90 - _convertSlopeToAngle(_getInnerCircleDerivative(_getInnerCircleX(AXLE_DISTANCE / 2)))
            elif wheel == 3:
                return 90 - _convertSlopeToAngle(_getOuterCircleDerivative(_getOuterCircleX(AXLE_DISTANCE / 2)))

        # Turning right
        else:
            if wheel == 0:
                return 90 - _convertSlopeToAngle(_getInnerCircleDerivative(_getInnerCircleX(AXLE_DISTANCE / 2)))
            elif wheel == 1:
                return  90 - _convertSlopeToAngle(_getOuterCircleDerivative(_getOuterCircleX(AXLE_DISTANCE / 2)))
            elif wheel == 2:
                return _convertSlopeToAngle(_getInnerCircleDerivative(_getInnerCircleX(AXLE_DISTANCE / 2))) + 90 
            elif wheel == 3:
                return _convertSlopeToAngle(_getOuterCircleDerivative(_getOuterCircleX(AXLE_DISTANCE / 2))) + 90

    # If we aren't trying to turn set all wheels to face forward
    else:
        return 0
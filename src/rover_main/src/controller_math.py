import math

"""Used to handle math for controlling the wheel differential"""

# Distance betwee the 2 sets of tires (meters)
TRACK_WIDTH = 1

# Distance between the front axel and back axel (meters)
AXLE_DISTANCE = 1

# Current radii of the circle to calculate angles from
circleRadius = 0


def updateCircleRadius(turnRadius):
    """Updates the inner circles radius"""
    b = 0.52345 / turnRadius
    circleRadius = ((0.5 * AXLE_DISTANCE) / math.sin(b))

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

def getWheelAngle(wheel):
    """Returns the required angle of the wheels enum"""
    if wheel == 0:
        return _convertSlopeToAngle(_getInnerCircleDerivative(_getInnerCircleX(AXLE_DISTANCE / 2))) + 90
    elif wheel == 1:
        return _convertSlopeToAngle(_getOuterCircleDerivative(_getOuterCircleX(AXLE_DISTANCE / 2))) + 90
    elif wheel == 2:
        return 90 - _convertSlopeToAngle(_getInnerCircleDerivative(_getInnerCircleX(AXLE_DISTANCE / 2)))
    elif wheel == 3:
        return 90 - _convertSlopeToAngle(_getOuterCircleDerivative(_getOuterCircleX(AXLE_DISTANCE / 2)))

    
        
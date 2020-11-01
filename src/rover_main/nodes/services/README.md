# Service Implementations

All code regarding services, specified bellow are service parameter calls
<br>
<br>


## I2C_Service:
---
<br>

**Arduino:**<br>
    To read digital data from a pin on an arduino: `dr <pin>`<br>
    To read analog data from a pin on an arduino: `ar <pin>`<br>
    <br>

**Adafruit 16-Channel PWM/Servo Driver:**<br>
    To set the PWM frequency of the device: `ada_pwm set_freq <frequency>`<br>
    To set the PWM range of a specific channel: `ada_pwm set_range <channel_num> <min_range> <max_range>`<br>
    To set the angle of a servo: `ada_pwm set_angle <channel_num> <angle>`<br>
    To set the range of movment of a servo: `ada_pwm set_angle_range <channel_num> <angle>`<br>
    To set the throttle of a motor: `ada_pwm set_power <channel_num> <power>`<br>
    **The Address for this device is 0x40 (64), however it doesnt really matter what you send to the service as the device address**


import time
from motorgo import BrakeMode, ControlMode, Plink
import math
import board
from light_utils import get_light
import adafruit_bh1750

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bh1750.BH1750(i2c)

plink = Plink()
left_motor = plink.channel1
right_motor = plink.channel2

left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY
left_motor.set_velocity_pid_gains(8, 0, 0)
right_motor.set_velocity_pid_gains(12, 1, 10)
plink.connect()

#PID vars
integral_error = 0
prev_error = 0
base_speed_left = 1.5 # rad/s
base_speed_right = -0.5
d_t = 0.01

#PID constants
Kp = 0.2
Ki = 0
Kd = .07

#will follow inside of circle, follow right side of line
lower_threshold = 33

upper_threshold = 45  #min light is 26, max light is 50

def get_light():
    
    """
    Reads and returns the ambient light intensity in lux.

    This function retrieves the current light level using the BH1750 light sensor.
    The BH1750 sensor measures the ambient light intensity and returns the value in lux.

    Returns:
        float: The measured light intensity in lux.
    """
    return sensor.lux


def line_follow_circle(prev_error, curr_light_val):
    
    if curr_light_val > upper_threshold:         
        error = curr_light_val - upper_threshold
    elif curr_light_val < lower_threshold: 
        error = lower_threshold - curr_light_val
    else: 
        error = 0
        
    #integral_error += error
    derivative_error = (error - prev_error)/d_t
    
    #calculate correction
    correction = Kp * error + Kd * derivative_error

    if curr_light_val > upper_threshold:  # Too much white, turn right
        print(f"Too much white: {curr_light_val} correction: {correction}\n")
        left_motor.velocity_command = base_speed_left + correction
        right_motor.velocity_command = base_speed_right - correction  # Increase reverse slightly
    
    elif curr_light_val < lower_threshold:  # Too much black, turn left
        print(f"Too much black: {curr_light_val} correction: {correction}\n")
        left_motor.velocity_command = base_speed_left - correction
        right_motor.velocity_command = base_speed_right + correction  # Reduce reverse
    else: 
        print(f"We're good at: {curr_light_val} correction: {correction}\n")
        left_motor.velocity_command = base_speed_left 
        right_motor.velocity_command = base_speed_right

    # if curr_light_val > upper_threshold: # turn right
    #     print(f"Too much white: {curr_light_val} correction: {correction}\n")
    #     left_motor.velocity_command = base_speed_left + correction
    #     right_motor.velocity_command = base_speed_right - correction  
    
    # elif curr_light_val < lower_threshold: #turn left
        
    #     left_motor.velocity_command = base_speed_left - correction
    #     right_motor.velocity_command = base_speed_right + correction

    
    prev_error = error #set error to prev
    return prev_error

    #print(f"light: {curr_light_val}, correction: {correction}, left velocity: {left_motor.velocity_command}, right_motor: {right_motor.velocity_command},   base_speed + correction: {base_speed - correction}")


while True:
    curr_light_val = get_light()  #get light value
    left_motor.velocity_command = base_speed_left
    right_motor.velocity_command = base_speed_right #spins in reverse
    print(f"Left: {left_motor.velocity}, Right: {right_motor.velocity}")
    #prev_error = line_follow_circle(prev_error, curr_light_val) #line follow

    time.sleep(d_t)  #short delay to stabilize loop

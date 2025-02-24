from motorgo import Plink, ControlMode
import time
import board
import adafruit_bh1750
import math
from light_utils import get_light
from line_following import Line_follower
from odometry import Odometry
from localization import Localization
from tof import read_data
import numpy as np

#Plink constants
velocity_kp = -8 #double check if this should still be negative
velocity_ki = 0
velocity_kd = 0

#line following constants
line_following_kp = 0.5
line_following_ki = 0
line_following_kd = 0.025

left_radius = 16.4875 #radii that the each respective wheel roughly travels around (inches)
right_radius = 5.9375

left_to_light_sensor = 5.125 # distance from each respective wheel to center of light sensor
right_to_light_sensor = 5.3125

min_velocity = 3

lower_threshold = 69.9 #lower light threshold
upper_threshold = 70.1 #upper light threshold

#odometry constants (inches)
wheel_base = 10.5 #if undershoot, decrease
#prev 10.55
wheel_diameter = 1.215 #if undershoot, increase 
#prev 1.2

x_curr =  -10.75#center of wheel base radius from center when you start
y_curr = 0
theta_curr = math.pi/2

#miscellanious constants
d_t = 0.03
panic_time = 10 


#Initialize Plink and set velocity command PID constants
plink = Plink()
left_motor = plink.channel1 # negative velocity is forward
right_motor = plink.channel2 

left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY

left_motor.set_velocity_pid_gains(velocity_kp, velocity_ki, velocity_kd)
right_motor.set_velocity_pid_gains(velocity_kp, velocity_ki, velocity_kd)

left_motor.velocity_command = 0
right_motor.velocity_command = 0

plink.connect()

#initialize i2c, light sensor, and distance sensor
i2c = board.I2C()  # uses board.SCL and board.SDA
light_sensor = adafruit_bh1750.BH1750(i2c)

#initialize line following object
motor_speed_ratio = left_radius/right_radius
light_sensor_distance_ratio = left_to_light_sensor/right_to_light_sensor
line_follower = Line_follower(left_motor, right_motor, line_following_kp, line_following_ki, line_following_kd, motor_speed_ratio, light_sensor_distance_ratio, min_velocity, lower_threshold, upper_threshold, d_t)


#main loop
if __name__ == "__main__":
    
    blocks_map = [1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    #GOAL REGION: 6
    

    #initialize objects
    odometry = Odometry(left_motor, right_motor, wheel_base, wheel_diameter, d_t, x_curr, y_curr, theta_curr)
    loc = Localization(2, block_threshold_upper=35, block_threshold_lower=5, blocks_map = blocks_map, goal_region = 6) 
    line_follower = Line_follower(left_motor, right_motor, line_following_kp, line_following_ki, line_following_kd, motor_speed_ratio, light_sensor_distance_ratio, min_velocity, lower_threshold, upper_threshold, d_t)

    #when high enough, go to goal
    goal_probability_threshold = 0.6


    start_time = time.time()
    while True:
        loop_time = time.time()-start_time
        light_val = get_light()

        #line follow
        line_follower.update_velocities(light_val)

        #update odometry, get angle, get tof data
        odometry.update()
        dist = read_data()
        current_angle = math.degrees(odometry.theta_curr) % 360

        if dist > 1: #not a bad value

            #integrated update combines motion update with sensor update, CHANGE CURRENT ANGLE
            belief = loc.integrated_update(dist, current_angle, sigma_region=5, sigma_sensor=2, d_expected=23)
            
            #gets index of most probable region
            most_probable_region = np.argmax(belief)

            #get target angle of most likely region
            target_angle = most_probable_region * loc.angles_per_region

            print(f"Probabilities: {belief}")#f"Current angle: {current_angle:.2f}°, "
              #f"Most probable region: {most_probable_region}, "
              
            

            #if we are adequately sure about best region and we've done at least 1 loop and were at the goal, STOP
            if belief[loc.goal_region] > goal_probability_threshold and loop_time > 30 and  loc.goal_region == loc.current_region:
                print(f"Probability {belief[loc.current_region]}, Stopped at Region {loc.current_region}")
                left_motor.velocity_command = 0
                right_motor.velocity_command = 0
                #STOP WHEELS
                break
                
            #print(loc.integrated_update(dist, current_angle=10, sigma_region=5, sigma_sensor=2, d_expected=23))

            #MIGHT NEED TO PLAY WITH SLEEP VALUE
        time.sleep(0.05) 















# #initialize probablistic localization object

# #thought 42.16, actual 43.375

# #loop for testing
# t = time.time()
# #while odometry.get_sector() < math.pi/2:
# while time.time() - t < 100:
#     #left_motor.velocity_command = -5
#     #right_motor.velocity_command = 5
#     line_follower.update_velocities(light_sensor.lux)
#     #print(light_sensor.lux)
#     odometry.update()
#     odometry.print_phi()
#     time.sleep(d_t)


"""
#main loop
t = time.Time()

while True:
    line_follower.update_velocities(light_sensor.lux) #update motor velocities based off sensor data
    odometry.update()
    odometry.print_sector()

    probablistic_thingy.update()
    probablistic_thingy.print_most_likely_sector()

    if probalistic_thingy.located() or time.time()-t >= 100-panic_time:
        sector = probalistic_thingy.get_most_likely_sector()
        while True:
            odometry.navigate_sector(sector)
            if odometry.get_sector() == sector:
            time.sleep(d_t)
                break
        break
    time.sleep(d_t)
"""
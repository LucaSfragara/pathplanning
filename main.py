from motorgo import Plink, ControlMode, BrakeMode
import time
import math
from odometry import nav_straight
from odometry import turn_in_place_dumb


plink = Plink()
    
left_motor = plink.channel1
right_motor = plink.channel2
left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY
left_motor.set_velocity_pid_gains(6, 1.4, 0.1)
right_motor.set_velocity_pid_gains(6, 1.4, 0.1)

plink.connect()

vertices = [(10, 10), (15, 10), (15, 15)]
currX, currY = 0, 0 #UPDATE BASED ON TAS
curr_angle = math.pi/2  #UPDATE BASED ON TAS

for goalX, goalY in vertices:

    currX, currY, curr_angle = turn_in_place_dumb(currX, currY, curr_angle, goalX, goalY)
    currX, currY, curr_angle = nav_straight(currX, currY, curr_angle, goalX, goalY)


from motorgo import Plink, ControlMode, BrakeMode
import time
import math
#left encoder is flipped


#= 0.6
#11.1-10.636 = 0.464
#11.2-10.749 = 0.451
#11.1-10.604 = 0.496
#11.75-11.075 = 0.675


#at power (1,1) our robot goes 23.1 inches in 3 seconds
#1 is maxPower, the lowest power our motors run at is 0.26
#we need to map command values which our 0-1 to 0.3-1 for our purpose

#formulas:
#x_new = x + v*cos * d_t
#y_new = y + v*sin * d_t
#theta_new = theta + w * d_t
#v_wheel  = w_wheel * r
#w_robot = (v_right - v_left) / L

plink = Plink()
left_motor = plink.channel1
right_motor = plink.channel2

#left_motor.control_mode = ControlMode.POWER
#right_motor.control_mode = ControlMode.POWER

left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY

velocity_p = -8
velocity_i = 0
velocity_d = 0
left_motor.set_velocity_pid_gains(velocity_p, velocity_i, velocity_d)
right_motor.set_velocity_pid_gains(velocity_p, velocity_i, velocity_d)
left_motor.velocity_command = 0
right_motor.velocity_command = 0


plink.connect()
right_constant = 1 #right wheel is weaker
wheelbase = 4.07 #inches #decrease if undershoot, increase if overshoot
#prev 4.110
d_t = .05
diameter = 1.76
straight_line_velocity = 3.5


def runge_kutta_step(x, y, theta, v, omega, dt):
    """Performs a single step of the Runge-Kutta method for odometry, correcting flipped axes."""
    k1_x = v * math.sin(theta)  # Swapped cos → sin
    k1_y = v * math.cos(theta) # Swapped sin → -cos
    k1_theta = omega

    k2_x = v * math.sin(theta + (dt / 2) * k1_theta)
    k2_y = v * math.cos(theta + (dt / 2) * k1_theta)
    k2_theta = omega

    k3_x = v * math.sin(theta + (dt / 2) * k2_theta)
    k3_y = v * math.cos(theta + (dt / 2) * k2_theta)
    k3_theta = omega

    k4_x = v * math.sin(theta + dt * k3_theta)
    k4_y = v * math.cos(theta + dt * k3_theta)
    k4_theta = omega

    x_new = x + (dt / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
    y_new = y + (dt / 6) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y)
    theta_new = theta + (dt / 6) * (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta)

    return x_new, y_new, theta_new


def odometry_fn():
    x, y, theta = 0, 0, 0 #initialize positions
    left_power = 0.38
    right_power = 0.38
    start_time = time.time()
    while time.time() - start_time <= (8):
        
        left_motor.power_command = left_power
        right_motor.power_command = -right_power

        v_left = (-left_motor.velocity * diameter/2)  
        v_right = (right_motor.velocity * diameter/2) 

        print(left_motor.velocity)

        v_forward = (v_left + v_right) / 2
        w_robot = (v_right - v_left) / wheelbase

        x, y, theta = runge_kutta_step(x, y, theta, v_forward, w_robot, d_t)

        theta = (theta + math.pi) % (2*math.pi) - math.pi

        time.sleep(d_t)

    return x,y,theta


def nav_straight(x_curr, y_curr, theta_curr, x_goal, y_goal): #straight_line velocity cld be ~4.12rad/s
    """Drives straight forward until distance to goal coordinates is increasing"""


    old_distance = math.sqrt((x_goal - x_curr)**2 + (y_goal - y_curr)**2)
    new_distance = old_distance
    
    #set motors to go straight forward
    left_motor.velocity_command = -straight_line_velocity
    right_motor.velocity_command = straight_line_velocity
    
    time.sleep(d_t)

    while new_distance <= old_distance+0.0: #may cause overshooting
        left_motor.velocity_command = -straight_line_velocity
        right_motor.velocity_command = straight_line_velocity

        #update odometry (x_curr, y_curr, theta_curr)
        v_left = (-left_motor.velocity * diameter/2)  
        v_right = (right_motor.velocity * diameter/2) 

        v_forward = (v_left + v_right) / 2
        w_robot = (v_right - v_left) / wheelbase

        x_curr, y_curr, theta_curr = runge_kutta_step(x_curr, y_curr, theta_curr, v_forward, w_robot, d_t)

        theta_curr = (theta_curr + math.pi) % (2*math.pi) - math.pi

        #update distances
        old_distance = new_distance
        new_distance = math.sqrt((x_goal - x_curr)**2 + (y_goal - y_curr)**2)
        
        #go sleepy times
        time.sleep(d_t)

    left_motor.velocity_command = 0
    right_motor.velocity_command = 0

    return x_curr, y_curr, theta_curr


def turn_in_place_PID(x_curr, y_curr, theta_curr, theta_goal):
    """takes current theta and turns to goal theta (both in rad)
        outputs resultant x, y, and theta"""
    #PID
    kp = 0.1
    ki = 0
    kd = 0

    min_error = 0.1
    

    curr_error = theta_goal-theta_curr
    prev_error = error

    happy = False

    while not happy:
        #sleepy time
        time.sleep(d_t)

        #update odometry (x_curr, y_curr, theta_curr)
        v_left = (-left_motor.velocity * diameter/2)  
        v_right = (right_motor.velocity * diameter/2) 

        v_forward = (v_left + v_right) / 2
        w_robot = (v_right - v_left) / wheelbase

        x_curr, y_curr, theta_curr = runge_kutta_step(x_curr, y_curr, theta_curr, v_forward, w_robot, d_t)

        theta_curr = (theta_curr + math.pi) % (2*math.pi) - math.pi
       
        #set turning values
        pid_value = kp*curr_error + ki*1 + kd*(curr_error-prev_error)
        left_motor.velocity_command(pid_value)
        right_motor.velocity_command(-pid_value)

        #update thetas
        prev_error = error
        error = theta_goal-theta_curr


        if curr_error < min_error: 
            happy = True
    
    left_motor.velocity_command(0)
    right_motor.velocity_command(0)

    return x_curr, y_curr, theta_curr
    

def turn_in_place_dumb(x_curr, y_curr, theta_curr, x_goal, y_goal): #might have a problem when you turn from positive to negative
    """takes current theta and turns to goal theta (both in rad)
        outputs resultant x, y, and theta"""
    
    theta_threshold = math.radians(2.4)
    
    #calculate d_theta
    d_x = x_goal - x_curr
    d_y = y_goal - y_curr

    angle_to_goal = math.atan2(d_y, d_x)
    
    d_theta = angle_to_goal - theta_curr
    
    # Normalize delta_theta to be within the range [-π, π]
    d_theta = (d_theta + math.pi) % (2 * math.pi) - math.pi

    #are we turning counter clockwise
    ccw = (d_theta > 0)
    direction = math.copysign(1, d_theta)

    #account for threshold
    d_theta = d_theta-direction*theta_threshold
    
    #are we done turning
    done = False

    curr_theta_turned = 0
    original_theta = theta_curr


    while not done:
        #set turning values
        left_motor.velocity_command = direction*straight_line_velocity
        right_motor.velocity_command = direction*straight_line_velocity
        

        #update odometry (x_curr, y_curr, theta_curr)
        v_left = (-left_motor.velocity * diameter/2)  
        v_right = (right_motor.velocity * diameter/2) 

        v_forward = (v_left + v_right) / 2
        w_robot = (v_right - v_left) / wheelbase

        x_curr, y_curr, theta_curr = runge_kutta_step(x_curr, y_curr, theta_curr, v_forward, w_robot, d_t)

        theta_curr = (theta_curr + math.pi) % (2*math.pi) - math.pi

        curr_theta_turned = theta_curr - original_theta

        if ccw:
            if curr_theta_turned >= d_theta:
                done = True
        elif curr_theta_turned <= d_theta:
            done = True

        #sleepy time
        time.sleep(d_t)
    
    left_motor.velocity_command = 0
    right_motor.velocity_command = 0

    print(math.degrees(theta_curr))

    return x_curr, y_curr, theta_curr


def tune_wheelbase():
    goal = math.pi/2 - math.radians(2.4)
    x_curr = 0
    y_curr = 0
    theta_curr = 0
    while theta_curr < goal:
        left_motor.velocity_command = straight_line_velocity
        right_motor.velocity_command= straight_line_velocity

        #update odometry (x_curr, y_curr, theta_curr)
        v_left = (-left_motor.velocity * diameter/2)  
        v_right = (right_motor.velocity * diameter/2) 

        v_forward = (v_left + v_right) / 2
        w_robot = (v_right - v_left) / wheelbase

        x_curr, y_curr, theta_curr = runge_kutta_step(x_curr, y_curr, theta_curr, v_forward, w_robot, d_t)

        theta_curr = (theta_curr + math.pi) % (2*math.pi) - math.pi

        time.sleep(d_t)

        print(math.degrees(theta_curr))

#print(turn_in_place_dumb(10, 0, 0, 1, 1))




from motorgo import Plink, ControlMode
import board
import adafruit_bh1750


class Line_follower:
    def __init__(self, left_motor, right_motor, line_following_kp, line_following_ki, line_following_kd, motor_speed_ratio, light_sensor_distance_ratio, min_velocity, lower_threshold, upper_threshold, d_t):
        self.left_motor = left_motor # Store left motor
        self.right_motor = right_motor # Store right motor
        self.line_following_kp = line_following_kp  # Proportional gain
        self.line_following_ki = line_following_ki  # Integral gain
        self.line_following_kd = line_following_kd  # Derivative gain
        self.motor_speed_ratio = motor_speed_ratio  # Motor speed ratio
        self.light_sensor_distance_ratio = light_sensor_distance_ratio #ratio for correction
        self.min_velocity = min_velocity  # min_velocity
        self.lower_threshold = lower_threshold #lower threshold for light sensor
        self.upper_threshold = upper_threshold #upper threshold for light sensor
        self.d_t = d_t #delay value for calculating derivative error
        
        self.integral_error = 0
        self.prev_error = 0

    def update_velocities(self, curr_light_val):
        Kp = self.line_following_kp #save time by copying some constants and objects
        Ki = self.line_following_ki
        Kd = self.line_following_kd
        lower_threshold = self.lower_threshold
        upper_threshold = self.upper_threshold
        ratio = self.motor_speed_ratio
        c_ratio = self.light_sensor_distance_ratio #correction ratio

        base_speed = self.min_velocity
        left_motor = self.left_motor
        right_motor = self.right_motor
        
        if curr_light_val > upper_threshold:         
            error = curr_light_val - upper_threshold
        elif curr_light_val < lower_threshold: 
            error = lower_threshold - curr_light_val
        else: 
            error = 0

        self.integral_error += error
        derivative_error = (error - self.prev_error)/self.d_t
        
        #calculate correction
        correction = Kp * error + Ki * self.integral_error + Kd * derivative_error

        # MAKE SURE ALL THE NEFATIVES ARE RIGHT HERE
        #following outside edge
        if curr_light_val > upper_threshold: # turn left
            #print(f"Too much white: {curr_light_val} correction: {correction}\n")
            left_motor.velocity_command = -ratio*base_speed + c_ratio*correction
            #print(f"left_motor velocity: {-ratio*base_speed - c_ratio*correction}")
            right_motor.velocity_command = base_speed + correction
            #print(f"right_motor velocity: {base_speed - correction}")
        
        elif curr_light_val < lower_threshold: #turn right
            #print(f"Too much black: {curr_light_val} correction: {correction}\n")
            left_motor.velocity_command = -ratio*base_speed - c_ratio*correction
            #print(f"left_motor velocity: {-ratio*base_speed + c_ratio*correction}")
            right_motor.velocity_command = base_speed - correction
            #print(f"right_motor velocity: {base_speed + correction}")
        else: 
            #print(f"We're good at: {curr_light_val} correction: {correction}\n")
            left_motor.velocity_command = -ratio*base_speed 
            right_motor.velocity_command = base_speed
        
        self.prev_error = error #set error to prev


import math
from motorgo import Plink, ControlMode

class Odometry:
    def __init__(self, left_motor, right_motor, wheel_base, wheel_diameter, d_t, x_curr, y_curr, theta_curr):
        self.left_motor = left_motor  # Store left motor
        self.right_motor = right_motor  # Store right motor
        self.wheel_base = wheel_base  # Distance between the wheels
        self.wheel_diameter = wheel_diameter  # Diameter of the wheels
        self.d_t = d_t  # time between steps
        self.x_curr = x_curr #starting odometry
        self.y_curr = y_curr
        self.theta_curr = theta_curr
     
    def update(self):
        wheelbase = self.wheel_base #copy some values to save space 
        diameter = self.wheel_diameter
        d_t = self.d_t
        
        x_curr = self.x_curr 
        y_curr = self.y_curr
        theta_curr = self.theta_curr


        v_left = (-self.left_motor.velocity * diameter/2) # make sure this negative is correct
        v_right = (self.right_motor.velocity * diameter/2) 

        v_forward = (v_left + v_right) / 2
        w_robot = (v_right - v_left) / wheelbase

        x_curr, y_curr, theta_curr = runge_kutta_step(x_curr, y_curr, theta_curr, v_forward, w_robot, d_t)

        theta_curr = (theta_curr + math.pi) % (2*math.pi) - math.pi

        self.x_curr, self.y_curr, self.theta_curr = x_curr, y_curr, theta_curr

    def get_sector(self):
        return self.theta_curr
    
    def print_sector(self):
        print(self.theta_curr)

    def update_and_print(self):
        self.update()
        print(f"X: {self.x_curr}, Y: {self.y_curr}, Theta: {math.degrees(self.theta_curr)}")
        
        

def runge_kutta_step(x, y, theta, v, omega, dt):
    """Performs a single step of the Runge-Kutta method for odometry."""
    k1_x = v * math.cos(theta)
    k1_y = v * math.sin(theta)
    k1_theta = omega

    k2_x = v * math.cos(theta + (dt / 2) * k1_theta)
    k2_y = v * math.sin(theta + (dt / 2) * k1_theta)
    k2_theta = omega

    k3_x = v * math.cos(theta + (dt / 2) * k2_theta)
    k3_y = v * math.sin(theta + (dt / 2) * k2_theta)
    k3_theta = omega

    k4_x = v * math.cos(theta + dt * k3_theta)
    k4_y = v * math.sin(theta + dt * k3_theta)
    k4_theta = omega

    x_new = x + (dt / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
    y_new = y + (dt / 6) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y)
    theta_new = theta + (dt / 6) * (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta)

    return x_new, y_new, theta_new
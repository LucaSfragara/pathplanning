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

left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER
plink.connect()
right_constant = 1 #right wheel is weaker
wheelbase = 4.110 #inches
d_t = 0.05
diameter = 1.78


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

x, y, theta = odometry_fn()
#x is actually our -y, y is actually our x
print(f"X position: {round(-y, 3)}, Y position: {round(x,3)}, Theta: {round(math.degrees(theta), 3)}")

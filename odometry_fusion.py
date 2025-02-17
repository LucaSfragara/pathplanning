import time
from motorgo import BrakeMode, ControlMode, Plink
import math
import numpy as np
from construct_graph import djistra_shortest_path
from construct_map import construct_map


plink = Plink()
    
left_motor = plink.channel1
right_motor = plink.channel2
left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY
left_motor.set_velocity_pid_gains(6, 1.4, 0.1)
right_motor.set_velocity_pid_gains(6, 1.4, 0.1)

imu = plink.imu
plink.connect()
# The IMU object provides the raw IMU data:
# - 3-axis accelerometer data in m/s^2 (linear acceleration)
# - 3-axis gyroscope data in rad/s (angular velocity)

map = construct_map(True, 4) #construct map for easy obstacles
goalX, goalY = (djistra_shortest_path(map, None))[-1] #get last coords in List??
at_goal = False #are we at end goal
d_t = 0.01
diameter = 2.2 #inches
wheelbase = 6.1875 #inches

K = .98 #angle complementary filter constant
M = 0.8 #x-position filter constant
N = 0.8 #y-position filter constant
alpha = 0.1 #low pass filter constant
beta = 0.1 #low pass filter constant for gyro
speed = 4

current_x, current_y = 0
#initialize encoder odometry values
encoder_velocity_left = 0
encorder_velocity_right = 0
v_forward = 0
encoder_robot_w = 0
encoder_x = 0
encoder_y = 0
encoder_theta = 0

#initialize accelerometer odometry values
accel_v_x = 0 
accel_v_y = speed #initial velocity command MAYBE CHANGE
accel_p_x = 0 #CHANGE BASED ON TAS
accel_p_y = 0 #CHANGE BASED ON TAS

ax = imu.accel[0] #order is x, y, z (left-right on our robot)
ay = imu.accel[1] #order is x, y, z (straight and behind on our robot)
az = imu.accel[2] #order is x, y, z (DONT NEED)
    
gx = imu.gyro[0] #order is x, y, z (DONT NEED)
gy = imu.gyro[1] #order is x, y, z (DONT NEED)
gz = imu.gyro[2] #order is x, y, z (WE ONLY NEED gz)

filtered_ax = ax
filtered_ay = ay

accel_theta = math.atan2(ay, ax)
gyro_theta = gz
filtered_gz = gz

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


def odometry_fusion(ax, ay, gz, accel_v_x, accel_v_y, accel_p_x, accel_p_y, gyro_theta):
    #apply low pass filter to ax, ay to filter out noise
    filtered_ax = alpha * ax + (1 - alpha) * filtered_ax
    filtered_ay = alpha * ay + (1 - alpha) * filtered_ay

    #encoder odometry code
    encoder_velocity_left = (-left_motor.velocity * diameter/2)  
    encorder_velocity_right = (right_motor.velocity * diameter/2)
    v_forward = (encoder_velocity_left + encorder_velocity_right) / 2
    encoder_robot_w = (encoder_velocity_left - encorder_velocity_right) / wheelbase
    encoder_x, encoder_y, encoder_theta = runge_kutta_step(encoder_x, encoder_y, encoder_theta, v_forward, encoder_robot_w, d_t)
    encoder_theta = (encoder_theta + math.pi) % (2*math.pi) - math.pi

    #accelerometer odometry
    accel_v_x += (filtered_ax * d_t) #integrate accel to get velocity
    accel_v_y += (filtered_ay * d_t) #same thing
    accel_p_x += (accel_v_x * d_t) #integrate velocity to get position
    accel_p_y += (accel_v_y * d_t) #integrate velocity to get position
    accel_theta = math.radians(math.atan2(ay, ax)) #get theta in radians NOT VERY ACCURATE

    #gyro odometry values
    filtered_gz = beta * gz + (1 - beta) * filtered_gz
    gyro_theta += (filtered_gz * d_t)  #get theta using gyro
    gyro_theta = (gyro_theta + math.pi) % (2 * math.pi) - math.pi

    #complimentary filter for gyro, accel angle, and then encoder position and accel position
    current_angle = K * (gyro_theta) + (1 - K) * (accel_theta) 
    current_angle = (current_angle + math.pi) % (2 * math.pi) - math.pi
    current_x = M * (encoder_x) + (1-M) * (accel_p_x)
    current_y = N * (encoder_y) + (1-M) * (accel_p_y)
    return [(current_x, current_y), current_angle, (accel_v_x, accel_v_y), (accel_p_x, accel_p_y), gyro_theta]


while not at_goal:
    left_motor.velocity_command = speed
    right_motor.velocity_command = -speed
    
    #get sensor readings
    ax = imu.accel[0] 
    ay = imu.accel[1] 
    gz = imu.gyro[2] 
    
    #get curr x and curr y, vx, vy, px, py, gyro theta
    odo_values = odometry_fusion(ax, ay, gz, accel_v_x, accel_v_y, accel_p_x, accel_p_y, gyro_theta)
    current_X, current_y = odo_values[0]
    current_angle = odo_values[1]
    accel_v_x, accel_v_y = odo_values[2]
    accel_p_x, accel_p_y = odo_values[3]
    gyro_theta = odo_values[4]


    #if at goal, stop looping
    if current_x == goalX and current_y == goalY:
        at_goal = True

    #d_t = time.time() - start_time #dynamically adjust d_t WE CAN CHANGE THIS
    print(f"X: {'%.4f' % current_x}, Y: {'%.4f' % current_y}, 0: {'%.4f' % current_angle}")
    time.sleep(d_t)

print(f"X: {'%.4f' % current_x}, Y: {'%.4f' % current_y}")


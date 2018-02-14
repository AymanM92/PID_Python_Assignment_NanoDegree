# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The desired trajectory for the 
# robot is the x-axis. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau * crosstrack_error
#
# You'll only need to modify the `run` function at the bottom.
# ------------
 
from random import *
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------
# 
# this is the Robot class
#

class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = float(x)
        self.y = float(y)
        self.orientation = float(orientation) % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = float(steering_noise)
        self.distance_noise = float(distance_noise)

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = float(drift)

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = gauss(steering, self.steering_noise)
        distance2 = gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)

############## ADD / MODIFY CODE BELOW ####################
# ------------------------------------------------------------------------
#
# run - does a single control run

def make_robot():
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(0, 1, 0)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot

robot = Robot()
robot.set(0, 1, 0)

def run(robot, tau_p,tau_d,tau_i, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    old_y = []
    sum_y = []
    prev_cte = robot.y
    sum_cte = 0
    #robot.set_steering_drift(10*np.pi/180)
    # TODO: your code here
    for i in range(n):
        cte = robot.y
        diff_cte= cte - prev_cte
        sum_cte += cte
        prev_cte = cte

        steering_wheel_angle = (-tau_p*cte)  - (tau_d*diff_cte) - (tau_i*sum_cte)
        #print(cte,diff_cte,sum_cte)
        robot.move(steering_wheel_angle,speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        print(robot,steering_wheel_angle)#,'==> Y old and y now',old_y,robot.y)

    #end of my code
    return x_trajectory, y_trajectory
    
x_trajectory, y_trajectory = run(robot, 0.2, 3, 0.004)
n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1)#, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='PID controller Tuned without twiddle Algorithem')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
ax1.legend()
#plt.show()
robot = make_robot()
x_trajectory, y_trajectory = run(robot, 2.9229268964347743, 10.326767087320677, 0.4932708323372665)
n = len(x_trajectory)
#fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax2.plot(x_trajectory, y_trajectory, 'b', label='PID controller Tuned with twiddle Algorithem')
ax2.plot(x_trajectory, np.zeros(n), 'r', label='reference')
ax2.legend()
plt.show()

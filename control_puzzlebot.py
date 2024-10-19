#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32

# Output messages and state variables
ControlMsg = Twist()
target = Point()  # Start with an empty target
arrived = False  # Indicates whether the car reached the target

wl = 0  # Left wheel velocity
wr = 0  # Right wheel velocity

# Control gains (adjust if necessary)
k_linear = 1.5
k_angular = 4.0
ki_linear = 0.02  # Integral gain for linear control

# Robot parameters
r = 0.05  # Wheel radius
l = 0.19  # Distance between wheels

# Position and orientation
pos_x = 0
pos_y = 0
theta = 0

# Error integrals
sum_error_linear = 0

def init_control_msg():
    """Initialize control message with zero velocities."""
    ControlMsg.linear.x = 0
    ControlMsg.angular.z = 0

def stop():
    """Stop the robot by setting velocities to zero."""
    ControlMsg.linear.x = 0
    ControlMsg.angular.z = 0
    Control_pub.publish(ControlMsg)
    rospy.loginfo("Reached the target. Waiting for new commands...")

def wl_callback(msg):
    """Callback to update left wheel velocity."""
    global wl
    wl = msg.data

def wr_callback(msg):
    """Callback to update right wheel velocity."""
    global wr
    wr = msg.data

def target_callback(msg):
    """Callback to receive new target coordinates."""
    global target, arrived
    target = msg  # Update the target
    arrived = False  # Reset the arrival flag
    rospy.loginfo(f"New target received: {target}")

def wrap_to_PI(angle):
    """Wrap the angle to the range [-pi, pi]."""
    result = np.fmod(angle + np.pi, 2 * np.pi)
    if result < 0:
        result += 2 * np.pi
    return result - np.pi

if __name__ == '__main__':
    rospy.init_node("closed_loop_controller")
    init_control_msg()

    # Set the loop rate
    loop_rate = rospy.Rate(20)  # 20 Hz control loop
    rospy.on_shutdown(stop)

    # Publishers and Subscribers
    Control_pub = rospy.Publisher("/puzzlebot/cmd_vel", Twist, queue_size=1)
    pos_x_pub = rospy.Publisher("/puzzlebot/pos_x", Float32, queue_size=1)
    pos_y_pub = rospy.Publisher("/puzzlebot/pos_y", Float32, queue_size=1)

    rospy.Subscriber('/puzzlebot/wl', Float32, wl_callback, queue_size=1)
    rospy.Subscriber('/puzzlebot/wr', Float32, wr_callback, queue_size=1)
    rospy.Subscriber('/target', Point, target_callback, queue_size=1)

    last_time = rospy.Time.now().to_sec()

    try:
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            dt = current_time - last_time

            # Update position and orientation using wheel velocities
            theta = wrap_to_PI(theta + r * (wr - wl) / l * dt)
            pos_x += r * (wr + wl) / 2 * dt * np.cos(theta)
            pos_y += r * (wr + wl) / 2 * dt * np.sin(theta)

            # Publish current position
            pos_x_pub.publish(pos_x)
            pos_y_pub.publish(pos_y)

            # Check if there's an active target and the car hasn't arrived yet
            if not arrived:
                # Calculate distance and angle errors
                error_distance = np.sqrt((target.x - pos_x)**2 + (target.y - pos_y)**2)
                error_angle = wrap_to_PI(np.arctan2(target.y - pos_y, target.x - pos_x) - theta)

                # Accumulate the linear error (with clamping to prevent windup)
                sum_error_linear = np.clip(sum_error_linear + error_distance * dt, -10, 10)

                # Control law (P control for angular, PI control for linear)
                linear_velocity = k_linear * error_distance + ki_linear * sum_error_linear
                angular_velocity = k_angular * error_angle

                # Stop if close enough to the target
                if error_distance < 0.05:
                    arrived = True
                    stop()
                else:
                    # Apply velocity limits to avoid instability
                    linear_velocity = np.clip(linear_velocity, 0, 0.5)
                    angular_velocity = np.clip(angular_velocity, -1.0, 1.0)

                    # Set the control message and publish it
                    ControlMsg.linear.x = linear_velocity
                    ControlMsg.angular.z = angular_velocity
                    Control_pub.publish(ControlMsg)

            # Update last time and sleep
            last_time = current_time
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass

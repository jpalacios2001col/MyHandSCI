#!/usr/bin/env python3

import rospy
import time
import myhand_sci_msgs.msg
import geometry_msgs.msg        
import std_msgs.msg
import transforms3d as tf
import numpy as np
import time
import argparse  # Added argparse module for command-line arguments

class MyHandSCI_Interface(object):
    def __init__(self, motor_velocity, motor_upper_bound):  # Added motor_velocity and motor_upper_bound as arguments
        rospy.init_node('computer_node', anonymous=True)
        self._motor_cmd_pub = rospy.Publisher('/device_node/motor_cmds', myhand_sci_msgs.msg.Motor_cmd, queue_size=10)
        self._angle_pub = rospy.Publisher('angle', std_msgs.msg.Float64, queue_size=10)
        self._motor_upper_bound_pub = rospy.Publisher('/device_node/motor_upper_bound', std_msgs.msg.Int64, queue_size=10)  # New publisher for motor_upper_bound
        
        # Subscribers
        rospy.Subscriber('/device_node/IMUs', geometry_msgs.msg.QuaternionStamped, self.imu_sub_callback)

        # Quaternions:
        self.imu_0_quaternion = (0, 0, 0, 0)
        self.imu_1_quaternion = (0, 0, 0, 0)

        # Initialize angle:
        self.angle = 0.0

        # Control Settings:
        self.angle_up_threshold = 15
        self.angle_low_threshold = -15

        # Device Settings:
        self.motor_velocity = motor_velocity  # Set motor_velocity from command-line argument
        self.motor_upper_bound = motor_upper_bound  # Set motor_upper_bound from command-line argument

        # Exponential smoothing parameters
        self.alpha = 0.2  # Smoothing factor (adjust as needed)
        self.prev_angle = 0.0

    def publish_cmd(self):
        if (self.angle > self.angle_up_threshold):
            input_cmd = self.motor_velocity
        elif (self.angle < self.angle_low_threshold):
            input_cmd = -self.motor_velocity
        else:
            input_cmd = 0

        cmd_msg = myhand_sci_msgs.msg.Motor_cmd()
        rospy.loginfo(f"Motor Command: {input_cmd}")
        if not rospy.is_shutdown():
            cmd_msg.cmd = input_cmd
            self._motor_cmd_pub.publish(cmd_msg)

            # Publish motor_upper_bound every time publish_cmd is called
            self.publish_motor_upper_bound(self.motor_upper_bound)

    def publish_angle(self, angle):
        angle_msg = std_msgs.msg.Float64()
        rospy.loginfo(f"Angle Published: {angle}")
        if not rospy.is_shutdown():
            angle_msg.data = angle
            self._angle_pub.publish(angle_msg)

    def publish_motor_upper_bound(self, value):  # New method for publishing motor_upper_bound
        motor_upper_bound_msg = std_msgs.msg.Int64()
        rospy.loginfo(f"Motor Upper Bound Published: {value}")
        if not rospy.is_shutdown():
            motor_upper_bound_msg.data = value
            self._motor_upper_bound_pub.publish(motor_upper_bound_msg)

    def imu_sub_callback(self, data):
        # Extract imu data:
        quaternion = (data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w)
        imu_id = data.header.frame_id

        # Record data for each IMU:
        if imu_id == "0":
            self.imu_0_quaternion = quaternion

        if imu_id == "1":
            self.imu_1_quaternion = quaternion

        # Calculate and display angle:
        self.angle = self.calculate_angle()

        # Publish angle and command:
        self.publish_angle(self.angle)
        self.publish_cmd()

    def calculate_angle(self):
        # Convert quaternions to rotation matrices
        R0 = tf.quaternions.quat2mat(self.imu_0_quaternion)
        R1 = tf.quaternions.quat2mat(self.imu_1_quaternion)

        # Get local Z-axis for each IMU:
        z_axis0 = R0[:, 2]
        z_axis1 = R1[:, 2]

        # Calculate angle with dot product:
        dot_product = np.dot(z_axis0, z_axis1)
        angle_radians = np.arccos(np.clip(dot_product, -1.0, 1.0))

        # Determine sign from cross product:
        orientation = np.sign(np.cross(z_axis0, z_axis1)[1])
        angle_degrees = np.degrees(angle_radians) * orientation

        # Apply exponential smoothing
        smoothed_angle = self.alpha * angle_degrees + (1 - self.alpha) * self.prev_angle
        self.prev_angle = smoothed_angle

        return smoothed_angle

def parse_args():
    parser = argparse.ArgumentParser(description='MyHandSCI Interface Script')
    parser.add_argument('--motor_velocity', type=int, default=150, help='Motor velocity value')
    parser.add_argument('--motor_upper_bound', type=int, default=5000, help='Motor upper bound value')
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    robot_interface = MyHandSCI_Interface(args.motor_velocity, args.motor_upper_bound)
    rospy.spin()

#!/usr/bin/env python3

import rospy
import time
import myhand_sci_msgs.msg
import geometry_msgs.msg
import transforms3d as tf
import numpy as np
import time

class MyHandSCI_Interface(object):
    def __init__(self): 
        rospy.init_node('computer_node', anonymous=True)
        self._motor_cmd_pub = rospy.Publisher('motor_cmds', myhand_sci_msgs.msg.Motor_cmd, queue_size=10)

        #subscribers
        rospy.Subscriber('IMUs', geometry_msgs.msg.QuaternionStamped, self.imu_sub_callback)
        rospy.sleep(1.0)

        self.imu_0_quaternion
        self.imu_1_quaternion

        time.sleep(3.0)

    def publish_cmd(self, input_cmd):
        cmd_msg = myhand_sci_msgs.msg.Motor_cmd()
        if not rospy.is_shutdown():
            cmd_msg.cmd = input_cmd
            self._motor_cmd_pub.publish(input_cmd)

    def imu_sub_callback(self, data):
        # Extract imu data:
        quaternion = (data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w)
        imu_id = data.header.frame_id

        # Record data for each IMU:
        if (imu_id == "0"):
         self.imu_0_quaternion = quaternion
         rospy.loginfo("IMU 0: ")
         rospy.loginfo(self.imu_0_quaternion)

        if (imu_id == "1"):
         self.imu_1_quaternion = quaternion
         rospy.loginfo("IMU 1: ")
         rospy.loginfo(self.imu_1_quaternion)

        # Calculate and display angle:
        angle = self.calculate_angle()
        rospy.loginfo("Angle: ")
        rospy.loginfo(angle)

    def calculate_angle(self):
        # Convert quaternions to rotation matrices
        R0 = tf.quaternions.quat2mat(self.imu_0_quaternion)
        R1 = tf.quaternions.quat2mat(self.imu_1_quaternion)

        # Get local Z-axis for each IMU:
        z_axis0 = R0[:, 2]
        z_axis1 = R1[:, 2]

        # Calculate angle wih dot product:
        dot_product = np.dot(z_axis0, z_axis1)
        angle_radians = np.arccos(np.clip(dot_product, -1.0, 1.0))

        # Determine sign from cross product:
        orientation = np.sign(np.cross(z_axis0, z_axis1)[1])
        angle_degrees = np.degrees(angle_radians) * orientation

        return angle_degrees

        
if __name__ == '__main__':
    robot_interface = MyHandSCI_Interface()
    rospy.spin()


    # robot_interface.publish_cmd(100)
    # time.sleep(1)
    # robot_interface.publish_cmd(0)
    # # time.sleep(3)
    # # robot_interface.publish_cmd(100)
    # # time.sleep(3)
    # # robot_interface.publish_cmd(0)
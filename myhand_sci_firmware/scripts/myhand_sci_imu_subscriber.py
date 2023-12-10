#!/usr/bin/env python3

import rospy
import time
import myhand_sci_msgs.msg

class MyHandSCI_Interface(object):
    def __init__(self): 
        rospy.init_node('computer_node', anonymous=True)
        self._motor_cmd_pub = rospy.Publisher('motor_cmds', myhand_sci_msgs.msg.Motor_cmd, queue_size=10)

        rospy.sleep(1.0)

    def publish_cmd(self, input_cmd):
        cmd_msg = myhand_sci_msgs.msg.Motor_cmd()
        if not rospy.is_shutdown():
            cmd_msg.cmd = input_cmd
            self._motor_cmd_pub.publish(input_cmd)

        

if __name__ == '__main__':
    robot_interface = MyHandSCI_Interface()

    robot_interface.publish_cmd(100)
    time.sleep(3)
    robot_interface.publish_cmd(0)
    time.sleep(3)
    robot_interface.publish_cmd(100)
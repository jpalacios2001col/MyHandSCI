#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String, Float64
import myhand_sci_msgs.msg

class DataCollectorNode:
    def __init__(self):
        rospy.init_node('data_collector_node', anonymous=True)

        # Initialize the custom message with default values
        self.data_msg = myhand_sci_msgs.msg.DataEntry()
        self.set_default_values()

        # Set up subscribers
        rospy.Subscriber('/device_node/encoder_counts', Int32, self.encoder_callback)
        rospy.Subscriber('/angle', Float64, self.angle_callback)
        rospy.Subscriber('/flag', String, self.flag_callback)
        rospy.Subscriber('/target_force', Float64, self.target_force_callback)
        rospy.Subscriber('/futek_node/futek', myhand_sci_msgs.msg.Futek, self.futek_callback)

        # Set up publisher
        self.data_pub = rospy.Publisher('/data_entry', myhand_sci_msgs.msg.DataEntry, queue_size=10)

        # Print a message to indicate successful initialization
        rospy.loginfo("Data Collector Node initialized successfully")

        # Start the main loop for continuous data collection and publishing
        self.main_loop()

    def set_default_values(self):
        # Set default values for fields
        self.data_msg.header.stamp = rospy.Time.now()
        self.data_msg.flag = "None"
        self.data_msg.encoder = 0
        self.data_msg.angle = 0.0
        self.data_msg.target_force = 0.0
        self.data_msg.futek_load = 0.0

    def encoder_callback(self, data):
        self.data_msg.encoder = data.data

    def angle_callback(self, data):
        self.data_msg.angle = data.data

    def flag_callback(self, data):
        self.data_msg.flag = data.data # if data.data else "None"

    def target_force_callback(self, data):
        self.data_msg.target_force = data.data

    def futek_callback(self, data):
        # For /futek_node/futek, save only the load part as Float32
        self.data_msg.futek_load = data.load

    def publish_data(self):
        # Set the header timestamp
        self.data_msg.header.stamp = rospy.Time.now()

        # Publish the custom message
        self.data_pub.publish(self.data_msg)

    def main_loop(self):
        rate = rospy.Rate(50)  # Set the desired publishing rate (1 Hz in this example)
        while not rospy.is_shutdown():
            # Continuously publish data
            self.publish_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        collector = DataCollectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
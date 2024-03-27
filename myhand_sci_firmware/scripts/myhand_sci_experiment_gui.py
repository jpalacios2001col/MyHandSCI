#!/usr/bin/env python3

import sys
import os
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel, QRadioButton, QLineEdit, QComboBox, QPushButton, QHBoxLayout
from PyQt5.QtGui import QFont, QDoubleValidator
from PyQt5.QtCore import QTimer, QTime

import rospy
from std_msgs.msg import String, Float64

class ExperimentGUI(QWidget):
    def __init__(self):
        super(ExperimentGUI, self).__init__()

        self.init_ui()
        self.update_flag_info("NONE", "0", "Na")  # Initialize selected_trial
        self.target_force = 0
        self.decimals = 2
        self.selected_trial = "Na"  # Initialize selected_trial

        # Timer setup
        self.timer = QTimer(self)
        self.time_elapsed = QTime(0, 0)
        self.timer.timeout.connect(self.update_timer)

        # ROS setup
        rospy.init_node('experiment_node', anonymous=True)
        self.flag_publisher = rospy.Publisher('/flag', String, queue_size=10)
        self.target_force_publisher = rospy.Publisher('/target_force', Float64, queue_size=10)

        # QTimer for continuous publishing
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.publish_ros_data)
        self.publish_timer.start(1000)  # Set the timeout interval in milliseconds (e.g., 1000ms = 1s)

        # Rosbag recording variables
        self.rosbag_process = None
        self.rosbag_folder = "/home/joaquin/myhandsci_ws/src/MyHandSCI/data_collection/session_2/force_experiments/"

    def init_ui(self):
        self.layout = QVBoxLayout()

        # Title
        exp_label = QLabel("Experiment")
        exp_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.layout.addWidget(exp_label)

        # Buttons
        radio_layout = QGridLayout()

        buttons1 = ["ND_MAX", "ND_LF", "ND_MF", "ND_HF"]
        buttons2 = ["WD_MAX", "WD_LF", "WD_MF", "WD_HF"]

        self.button_group = []

        for row, btn_text in enumerate(buttons1):
            btn = QRadioButton(btn_text)
            self.button_group.append(btn)
            radio_layout.addWidget(btn, row, 0)
            btn.clicked.connect(lambda _, b=btn, r=row: self.handle_button_press(b, r))

        for row, btn_text in enumerate(buttons2):
            btn = QRadioButton(btn_text)
            self.button_group.append(btn)
            radio_layout.addWidget(btn, row, 1)
            btn.clicked.connect(lambda _, b=btn, r=row: self.handle_button_press(b, r))

        self.layout.addLayout(radio_layout)

        # Trial selection
        trial_label = QLabel("Trial")
        trial_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.layout.addWidget(trial_label)

        trial_combobox = QComboBox()
        trial_combobox.addItems(["Na", "1", "2", "3", "4"])  # Updated trial options
        trial_combobox.currentIndexChanged.connect(self.handle_trial_selection)
        self.layout.addWidget(trial_combobox)

        # Target Force
        target_label = QLabel("Max Force (N)")
        target_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.layout.addWidget(target_label)

        input_layout = QVBoxLayout()

        self.input_box = QLineEdit()
        self.input_box.setValidator(QDoubleValidator())  # Allow decimal input
        self.input_box.returnPressed.connect(self.handle_enter_pressed)
        input_layout.addWidget(self.input_box)

        self.display_box = QLabel()
        self.display_box.setFont(QFont("Arial", 14))
        input_layout.addWidget(self.display_box)

        self.layout.addLayout(input_layout)

        # Flag
        flag_label = QLabel("Flag:")
        flag_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.layout.addWidget(flag_label)

        self.flag_info = QLabel()
        self.flag_info.setFont(QFont("Arial", 14))
        self.layout.addWidget(self.flag_info)

        # Start Trial and Timer Section
        start_trial_label = QLabel("Start Trial and Timer:")
        start_trial_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.layout.addWidget(start_trial_label)

        # Start, End, Reset Buttons and Timer Display
        timer_layout = QHBoxLayout()

        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_trial)
        timer_layout.addWidget(self.start_button)

        self.end_button = QPushButton("End")
        self.end_button.clicked.connect(self.end_trial)
        timer_layout.addWidget(self.end_button)

        self.timer_display = QLabel("00:00:00")
        self.timer_display.setFont(QFont("Arial", 14))
        timer_layout.addWidget(self.timer_display)

        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_trial)
        timer_layout.addWidget(self.reset_button)

        self.layout.addLayout(timer_layout)

        self.setLayout(self.layout)
        self.setGeometry(100, 100, 500, 400)  # Set the window size
        self.show()

    def handle_button_press(self, button, row):
        # Unpress all other buttons
        for other_button in self.button_group:
            if other_button != button:
                other_button.setChecked(False)

        # Update flag information
        max_force = self.input_box.text()
        self.target_force = self.calculate_target_force(button.text(), max_force)
        self.update_flag_info(button.text(), self.target_force, self.selected_trial)

        # Publish flag and target force information to ROS
        self.publish_ros_data()

    def handle_enter_pressed(self):
        # Update the display box when Enter is pressed
        input_text = self.input_box.text()
        self.display_box.setText(input_text)

        # Publish flag and target force information to ROS
        self.publish_ros_data()

    def calculate_target_force(self, button_text, max_force):
        # Calculate target force based on the button
        if "MAX" in button_text:
            return round(float(max_force), self.decimals)
        elif "LF" in button_text:
            return round(float(max_force) * 0.2, self.decimals)
        elif "MF" in button_text:
            return round(float(max_force) * 0.5, self.decimals)
        elif "HF" in button_text:
            return round(float(max_force) * 0.8, self.decimals)
        else:
            return "0"

    def publish_ros_data(self):
        # Publish flag and target force to ROS
        flag_data = f"{self.flag_info.text()}"
        rospy.loginfo(f"Publishing Flag: {flag_data}")
        self.flag_publisher.publish(flag_data)

        rospy.loginfo(f"Publishing Target Force: {self.target_force}N")
        self.target_force_publisher.publish(self.target_force)

    def update_flag_info(self, pressed_button, target_force, trial):
        # Update the flag information
        self.flag_info.setText(f"{pressed_button};T:{target_force}N;Trial:{trial}")

    def handle_trial_selection(self, index):
        # Handle trial selection from the combobox
        trial_options = ["Na", "1", "2", "3", "4"]
        self.selected_trial = trial_options[index]

    def start_trial(self):
        # Start the timer
        self.timer.start(1000)  # Start the timer with 1-second interval

        # Start rosbag recording
        flag_name = self.flag_info.text().replace(";", "_").replace(":", "_")
        rosbag_filename = self.get_unique_rosbag_name(flag_name)
        rosbag_filepath = os.path.join(self.rosbag_folder, rosbag_filename)
        self.rosbag_process = subprocess.Popen(["rosbag", "record", "-O", rosbag_filepath, "/data_entry"])

    def end_trial(self):
        # Stop the timer
        self.timer.stop()  # Stop the timer when the "End" button is clicked

        # Stop rosbag recording
        if self.rosbag_process is not None:
            self.rosbag_process.terminate()
            self.rosbag_process.wait()

    def reset_trial(self):
        # Reset the timer
        self.time_elapsed = QTime(0, 0)
        self.timer_display.setText(self.time_elapsed.toString("hh:mm:ss"))

    def update_timer(self):
        # Update the timer display
        self.time_elapsed = self.time_elapsed.addSecs(1)
        self.timer_display.setText(self.time_elapsed.toString("hh:mm:ss"))

    def get_unique_rosbag_name(self, base_name):
        # Check if the file already exists, if yes, append a number
        rosbag_name = f"{base_name}.bag"
        count = 1
        while os.path.exists(os.path.join(self.rosbag_folder, rosbag_name)):
            rosbag_name = f"{base_name}_{count}.bag"
            count += 1
        return rosbag_name

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ExperimentGUI()
    sys.exit(app.exec_())

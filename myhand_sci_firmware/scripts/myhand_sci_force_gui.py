#!/usr/bin/env python3

import sys
import argparse
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, QFrame
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont
import rospy
import myhand_sci_msgs.msg

DEFAULT_TARGET_FORCE = 10  # Default target force value
DEFAULT_MAX_FORCE = 30  # Default maximum force value
DEFAULT_GREEN_RANGE = 1  # Default range within which the color is green

class ForceGUI(QWidget):
    def __init__(self, target_force, max_force, green_range):
        super(ForceGUI, self).__init__()

        self.target_force = target_force
        self.max_force = max_force
        self.green_range = green_range
        self.max_measured_force = 0.0  # Variable to store the maximum measured force

        self.init_ui()

        # ROS initialization
        rospy.init_node('force_gui_node', anonymous=True)
        rospy.Subscriber('/futek_node/futek', myhand_sci_msgs.msg.Futek, self.force_sub_callback)

        self.force = 0.0

    def init_ui(self):
        self.setWindowTitle('Force GUI')
        self.setGeometry(100, 100, 600, 900)  # Adjust size

        main_layout = QVBoxLayout()

        # Create a horizontal layout for progress bars
        progress_layout = QHBoxLayout()

        # Create the main progress bar with larger font size
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setOrientation(Qt.Vertical)  # Set orientation to vertical
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(self.max_force)
        self.progress_bar.setFormat('%v N')  # Display current force instead of percentage
        progress_bar_width = int(self.width()/2)  # Convert to integer
        self.progress_bar.setFixedWidth(progress_bar_width)
        progress_layout.addWidget(self.progress_bar)

        # Create a target progress bar next to the main progress bar
        self.target_progress_bar = QProgressBar(self)
        self.target_progress_bar.setOrientation(Qt.Vertical)  # Set orientation to vertical
        self.target_progress_bar.setMinimum(0)
        self.target_progress_bar.setMaximum(self.max_force)
        self.target_progress_bar.setFormat('%v N')  # Display target force instead of percentage
        self.target_progress_bar.setFixedWidth(progress_bar_width)
        progress_layout.addWidget(self.target_progress_bar)

        main_layout.addLayout(progress_layout)

        # Create the current force label with larger font size
        self.force_label = QLabel('Current Force: 0.0 N')
        font_force = QFont()
        font_force.setPointSize(35)  # Set the font size to 20 for the current force
        self.force_label.setFont(font_force)
        main_layout.addWidget(self.force_label)

        main_layout.addWidget(QFrame())  # Add a line of blank space at the bottom as a divider

        # Create a label to display the maximum force measured so far
        self.max_force_label = QLabel('Max Force Measured: 0.0 N')
        font_max_force = QFont()
        font_max_force.setPointSize(15)  # Set the font size for the maximum force label
        self.max_force_label.setFont(font_max_force)
        main_layout.addWidget(self.max_force_label)

        self.setLayout(main_layout)

        self.show()

    def force_sub_callback(self, force_msg):
        self.force = round(abs(force_msg.load), 2)  # Round to two decimal places

        # Update progress bars
        self.progress_bar.setValue(self.force)
        self.target_progress_bar.setValue(self.target_force)

        # Calculate the distance from the target force
        distance_to_target = abs(self.force - self.target_force)

        # Update the maximum measured force if the current force is greater
        if self.force > self.max_measured_force:
            self.max_measured_force = self.force

        # Update the maximum force label
        self.max_force_label.setText('Max Force Measured: {} N'.format(self.max_measured_force))

        # Map the distance to a color gradient
        color = self.map_distance_to_color(distance_to_target)

        # Set the foreground color of the main progress bar
        self.progress_bar.setStyleSheet("QProgressBar::chunk { background-color: " + color.name() + "; }")

        # Set the target progress bar always to green
        self.target_progress_bar.setStyleSheet("QProgressBar::chunk { background-color: " + QColor(20, 255, 0).name() + "; }")

        # Update the force label with larger font size and two decimal places
        self.force_label.setText(
            '   Current Force: {} N \t  Target Force: {} N'.format(self.force, self.target_force))

    def map_distance_to_color(self, distance):
        # Map the distance to a color gradient (red to orange to yellow)
        max_distance = self.max_force - self.target_force  # Maximum distance from target to maximum force
        normalized_distance = distance / max_distance

        # Interpolate between red and yellow outside the target range
        r = int(normalized_distance * (255 - 100)) + 100
        g = int((1 - normalized_distance) * 180)
        b = 0

        # If within the green range, set the color to green for the target progress bar
        if distance <= self.green_range:
            return QColor(20, 255, 0)  # Green
        else:
            return QColor(r, g, b)


def main():
    parser = argparse.ArgumentParser(description='ROS Node for Force GUI')
    parser.add_argument('--target_force', type=float, default=DEFAULT_TARGET_FORCE,
                        help='Target force value (default: {})'.format(DEFAULT_TARGET_FORCE))
    parser.add_argument('--max_force', type=float, default=DEFAULT_MAX_FORCE,
                        help='Maximum force value (default: {})'.format(DEFAULT_MAX_FORCE))
    parser.add_argument('--green_range', type=float, default=DEFAULT_GREEN_RANGE,
                        help='Range within which the color is green (default: {})'.format(DEFAULT_GREEN_RANGE))
                        
    args = parser.parse_args()

    app = QApplication(sys.argv)
    gui = ForceGUI(args.target_force, args.max_force, args.green_range)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

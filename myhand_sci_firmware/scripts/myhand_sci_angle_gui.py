#!/usr/bin/env python3

import sys
import argparse
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtCore import Qt
import rospy
import std_msgs.msg

class AngleGUI(QWidget):
    def __init__(self, threshold):
        super(AngleGUI, self).__init__()

        self.threshold = threshold

        self.init_ui()

        # ROS initialization
        rospy.init_node('angle_gui_node', anonymous=True)
        rospy.Subscriber('angle', std_msgs.msg.Float64, self.angle_sub_callback)

        self.angle = 0.0

    def init_ui(self):
        self.setWindowTitle('Angle GUI')
        self.setGeometry(100, 100, 800, 400)  # Increased the height

        main_layout = QVBoxLayout()

        # Create the progress bar with larger font size
        self.angle_progress_bar = QProgressBar(self)
        self.angle_progress_bar.setMinimum(int(-self.threshold * 3))
        self.angle_progress_bar.setMaximum(int(self.threshold * 3))
        self.angle_progress_bar.setFormat('%v degrees')
        self.angle_progress_bar.setValue(0)
        self.angle_progress_bar.setFixedHeight(150)
        self.angle_progress_bar.setStyleSheet("QProgressBar { font-size: 100px; }")
        main_layout.addWidget(self.angle_progress_bar)

        # Create three rectangles below the progress bar
        rectangles_layout = QHBoxLayout()

        red_rect = QLabel(self)
        red_rect.setStyleSheet("background-color: red;")
        red_rect.setFixedHeight(30)  # Decrease the height of the red rectangle
        rectangles_layout.addWidget(red_rect)

        grey_rect = QLabel(self)
        grey_rect.setStyleSheet("background-color: grey;")
        grey_rect.setFixedHeight(30)  # Decrease the height of the grey rectangle
        rectangles_layout.addWidget(grey_rect)

        green_rect = QLabel(self)
        green_rect.setStyleSheet("background-color: green;")
        green_rect.setFixedHeight(30)  # Decrease the height of the green rectangle
        rectangles_layout.addWidget(green_rect)

        main_layout.addLayout(rectangles_layout)

        # Create labels for CLOSE and OPEN
        labels_layout = QHBoxLayout()

        open_label = QLabel('OPEN', self)
        close_label = QLabel('CLOSE', self)

        open_label.setStyleSheet("font-size: 80px;")  # Increase font size
        close_label.setStyleSheet("font-size: 80px;")  # Increase font size

        labels_layout.addWidget(open_label, alignment=Qt.AlignLeft)
        labels_layout.addWidget(close_label, alignment=Qt.AlignRight)

        # Add labels to the main layout
        main_layout.addLayout(labels_layout)

        # Create a label for the threshold angle
        threshold_label = QLabel('Threshold Angle: {} degrees'.format(self.threshold), self)
        threshold_label.setStyleSheet("font-size: 20px;")
        main_layout.addWidget(threshold_label, alignment=Qt.AlignCenter)

        self.setLayout(main_layout)
        self.show()

    def angle_sub_callback(self, angle_msg):
        # Cap the absolute value of the angle at 3*threshold degrees
        self.angle = int(min(self.threshold * 3.0, max(-self.threshold * 3.0, round(angle_msg.data, 2))))

        print("Angle:", self.angle)

        # Update the progress bar
        self.angle_progress_bar.setValue(self.angle)

        # Determine the color based on the angle conditions
        color = self.map_angle_to_color(self.angle)

        print("Color:", color)

        # Set the foreground color of the progress bar
        self.angle_progress_bar.setStyleSheet("QProgressBar::chunk { background-color: " + color.name() + "; }")


    def map_angle_to_color(self, angle):
        # Map the angle to color conditions (grey, green, red)
        if -self.threshold < angle < self.threshold:
            return QColor(128, 128, 128)  # Grey for NEUTRAL
        elif angle >= self.threshold:
            return QColor(20, 255, 0)  # Green for CLOSE
        else:
            return QColor(255, 0, 0)  # Red for OPEN

def main():
    parser = argparse.ArgumentParser(description='ROS Node for Angle GUI')
    parser.add_argument('--threshold', type=float, default=15,
                        help='Threshold angle for NEUTRAL condition (default: 15)')

    args = parser.parse_args()

    app = QApplication(sys.argv)
    gui = AngleGUI(args.threshold)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

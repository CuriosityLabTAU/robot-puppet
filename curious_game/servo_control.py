import rospy
from std_msgs.msg import String
import numpy as np
import servo
import random


class ServoControl:

    def __init__(self):
        self.skeleton_angles = np.zeros([6])

    def start(self):
        #init a listener to kinect angles
        rospy.init_node('servo_control')
        rospy.Subscriber("skeleton_angles", String, self.callback)
        rospy.spin()

    def callback(self, data):
        self.skeleton_angles = np.array([float(x) for x in data.data.split(',')])
        servo.move(1, int(self.skeleton_angles[4] * (180 / np.pi)))
        servo.move(2, int(self.skeleton_angles[5] * (180 / np.pi)))
        servo.move(3, int(self.skeleton_angles[2] * (180 / np.pi)))


servo_control = ServoControl()
servo_control.start()

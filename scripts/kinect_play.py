#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Quaternion, Pose2D
import os
from playsound import playsound
import json


class KinectPlayback():
    def __init__(self):
        self.saved_points = {}
        self.path = rospkg.RosPack().get_path('lazarillo')
        self.key = ''

        self.sub = rospy.Subscriber('/lazarillo/buttons', Quaternion, self.callback)
        self.pub = rospy.Publisher('/lazarillo/goal', Pose2D, queue_size=1)

        self.read_points()

    def callback(self, data):
        if data.x or data.y or data.z or data.w:
            if self.btn_2_key(data) == self.key:
                goal = Pose2D()
                goal.x = self.saved_points[self.key][0]
                goal.y = self.saved_points[self.key][1]
                self.pub.publish(goal)
            else:
                self.key = self.btn_2_key(data)

                playsound(f'{self.path}/audio/eleccion.mp3')

                os.chdir(f'{self.path}/audio/{self.key}')
                os.system('cvlc --play-and-exit --run-time=2 --gain 7 channel1.wav 2> /dev/null')
                
                playsound(f'{self.path}/audio/confirmar.mp3')

    def btn_2_key(self, btn):
        if btn.x:
            return 'button_0'
        elif btn.y:
            return 'button_1'
        elif btn.z:
            return 'button_2'
        elif btn.w:
            return 'button_3'
        return ''

    def read_points(self):
        data_file = f'{self.path}/data/saved_points.json'
        with open(data_file) as json_file:
            self.saved_points = json.load(json_file)


if __name__ == '__main__':
    rospy.init_node('kinect_play')
    kplay = KinectPlayback()
    try:
        rospy.spin()
    except Exception as e:
        print(e)

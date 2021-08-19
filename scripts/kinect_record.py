#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Quaternion, Twist
import subprocess
from playsound import playsound
import json


class KinectMic():
    def __init__(self):
        self.sub = rospy.Subscriber('/lazarillo/buttons', Quaternion, self.callback)
        self.saved_points = {
            'button_0': [0, 0],
            'button_1': [0, 0],
            'button_2': [0, 0],
            'button_3': [0, 0]
        }
        self.path = rospkg.RosPack().get_path('lazarillo')

    def init_kinect(self):
        process = subprocess.Popen('freenect-wavrecord', cwd=f'{self.path}/audio/dummy',
            shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()

    def callback(self, data):
        if data.x or data.y or data.z or data.w:
            msg = rospy.wait_for_message('/lazarillo/odom', Twist, timeout=5)

            if data.x:
                key = 'button_0'
            elif data.y:
                key = 'button_1'
            elif data.z:
                key = 'button_2'
            elif data.w:
                key = 'button_3'

            playsound(f'{self.path}/audio/guardar.mp3')
            playsound(f'{self.path}/audio/beep.mp3')

            self.saved_points[key][0] = msg.linear.x
            self.saved_points[key][1] = msg.linear.y
            
            process = subprocess.Popen('freenect-wavrecord', cwd=f'{self.path}/audio/{key}',
                shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
            
            playsound(f'{self.path}/audio/beep.mp3')

    def stop(self):
        data_file = f'{self.path}/data/saved_points.json'
        with open(data_file, 'w') as outfile:
            json.dump(self.saved_points, outfile, indent=2)


if __name__ == '__main__':
    rospy.init_node('kinect_record')
    kmic = KinectMic()    
    kmic.init_kinect()
    rospy.on_shutdown(kmic.stop)
    try:
        rospy.spin()
    except Exception as e:
        print(e)

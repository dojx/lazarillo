#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import numpy as np

# Convertir quaternion a yaw
def quat_2_yaw(q):
    t1 = 2.0 * (q[3] * q[2] + q[0] * q[1])
    t2 = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    return np.arctan2(t1, t2)

class MapToBase:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.odom_pub = rospy.Publisher('/lazarillo/map_odom', Pose2D, queue_size=10)

        self.listener = tf.TransformListener()
    
    def callback(self, data):
        try:
            (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            pos = Pose2D()
            pos.theta = quat_2_yaw(rot)
            pos.x = trans[0] + np.cos(pos.theta) * 0.08
            pos.y = trans[1]
            # pos.x = trans[0] - np.cos(pos.theta) * 0.08
            # pos.y = trans[1] - np.sin(pos.theta) * 0.08
            self.odom_pub.publish(pos)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


if __name__ == '__main__':
    rospy.init_node('map_to_base', anonymous=True)
    map_base = MapToBase()
    try:
        rospy.spin()
    except Exception as e:
        print(e)
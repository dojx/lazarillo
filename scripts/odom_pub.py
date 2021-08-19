#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3
import math


class OdomPublisher():
    def __init__(self):
        self.sub = rospy.Subscriber('/lazarillo/odom', Twist, self.callback)
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=30)

        self.odom_broadcaster = tf.TransformBroadcaster()

    def callback(self, data):
        x = data.linear.x
        y = data.linear.y
        th = data.linear.z

        vx = data.angular.x
        vy = data.angular.y
        vth = data.angular.z

        current_time = rospy.Time.now()

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # x_off = 0.08 * math.cos(th)
        # y_off = 0.08 * math.sin(th)

        x_off = 0
        y_off = 0
        

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (x + x_off, y + y_off, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        self.pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('odom_publisher', anonymous=True)
    control = OdomPublisher()
    try:
        rospy.spin()
    except Exception as e:
        print(e)

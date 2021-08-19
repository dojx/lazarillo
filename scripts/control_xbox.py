#!/usr/bin/env python

# ROS librerias
import rospy
# ROS mensajes
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class Controller():
    def __init__(self):
        '''
        float32[] axes = [
            0 : left x-axis,
            1 : left y-axis,
            2 : left trigger
            5 : right trigger        
        ]

        int32[] buttons = [
            0 : A,
            1 : B,
            2 : X,
            3 : Y
        ]

        range = [1, -1]
        '''
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)
        self.pos_sub = rospy.Subscriber('/lazarillo/odom', Twist, self.callback2)
        self.pub = rospy.Publisher('/lazarillo/llantas_vel', Quaternion, queue_size=10)

        self.wl = 0
        self.wr = 0

        self.min_pwm = 100
        self.max_pwm = 200

    def callback(self, data): 
        vel = 0

        # Linear  
        if data.axes[5] and data.axes[5] < 1:
            vel = translate(data.axes[5], 1, -1, self.min_pwm, self.max_pwm)
        elif data.axes[2] and data.axes[2] < 1:
            vel = translate(data.axes[2], 1, -1, -self.min_pwm, -self.max_pwm) 

        # Angular
        if data.axes[0] > 0:
            self.wr = vel 
            self.wl = -vel
        elif data.axes[0] < 0:
            self.wl = vel
            self.wr = -vel
        else:
            self.wl = vel
            self.wr = vel

        

    def callback2(self, data):
        msg = Quaternion()    
        msg.x = abs(self.wl) # Left wheel pwm
        msg.y = abs(self.wr) # Right wheel pwm
        msg.z = self.wl >= 0 # Left wheel direction
        msg.w = self.wr >= 0 # Right wheel direction

        self.pub.publish(msg)


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


if __name__ == '__main__':
    rospy.init_node('control_xbox', anonymous=True)
    control = Controller()
    try:
        rospy.spin()
    except Exception as e:
        print(e)   
    


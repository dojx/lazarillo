# ROS librerias
import rospy
# ROS mensajes
from geometry_msgs.msg import Quaternion
import time
import os

if __name__ == '__main__':
    rospy.init_node('initialize')
    vel_pub = rospy.Publisher('/lazarillo/llantas_vel', Quaternion, queue_size=10)

    # msg = Quaternion()

    # time.sleep(1)

    # msg.x = 120
    # msg.y = 120
    # msg.z = 0
    # msg.w = 1
    
    # vel_pub.publish(msg)

    # time.sleep(2)

    # msg.z = 1
    # msg.w = 0
    
    # vel_pub.publish(msg)

    # time.sleep(3)

    # msg.z = 0
    # msg.w = 1
    
    # vel_pub.publish(msg)

    # time.sleep(1.8)

    # msg.x = 0
    # msg.y = 0
    # msg.z = 1
    # msg.w = 1
    
    # vel_pub.publish(msg)

    time.sleep(15)

    rospy.set_param('/rtabmap/Vis/MinInliers', '150')

    os.system('rosservice call /rtabmap/update_parameters')

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Byte, Int32, Float32


def arm_pub(joy_topic):
    arm_commands = int(0xFF)

    # Conversion control base rotation
    if (buttons[2] != 0):
        arm_commands ^= (1 << (2*3))
    elif (buttons[1] != 0):
        # arm_commands ^= (0 << (2*3))
        pass
    else:
        arm_commands ^= (2 << (2*3))

    # Conversion control upper arm extention
    if (joy_topic.axes[5] <= 0):
        arm_commands ^= (1 << (2*2))
    elif (joy_topic.buttons[5] != 0):
        # arm_commands ^= (0 << (2*2))
        pass
    else:
        arm_commands ^= (2 << (2*2))

    # Conversion control forearm extention
    if (joy_topic.axes[2] <= 0):
        arm_commands ^= (1 << (2*1))
    elif (joy_topic.buttons[4] != 0):
        # arm_commands ^= (0 << (2*1)
        pass
    else:
        arm_commands ^= (2 << (2*1))

    # Conversion control wrist rotation
    if (joy_topic.axes[6] < -0.25):
        arm_commands ^= (1 << (2*0))
    elif (joy_topic.axes[6] > 0.25):
        # arm_commands ^= (0 << (2*0))
        pass
    else:
        arm_commands ^= (2 << (2*0))
    
    arm_pub.publish(arm_commands)

def joy_subscriber(data):
    arm_pub(data)

def arm_control():
    arm_pub = rospy.Publisher('arm_topic', int16, queue_size=4)
    rospy.init_node('base', anonymous=False)
    rospy.Subscriber("Joy", Joy, joy_subscriber)
    rospy.spin()

if __name__ == '__main__':
    try:
        arm_control()
    except rospy.ROSInterruptException:
        pass

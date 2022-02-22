#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Empty
from sensor_msgs.msg import Joy

rospy.init_node('rover_control', anonymous=False)
pub = rospy.Publisher('chatter', Int16, queue_size=1)
arm_pub = rospy.Publisher('arm_topic', Int16, queue_size=1)
wheels_pub = rospy.Publisher('wheels_topic', Int16, queue_size=1)
timer_pub = rospy.Publisher('timeout_topic', Empty, queue_size=1)

def wheels_pub_function(joy_topic):
    global wheels_commands
    wheels_commands = int(0x00)

    left = joy_topic.axes[1] * 7
    right = joy_topic.axes[4] * 7 

    wheels_commands = (int(abs(left))<<4) + int(abs(right))

    if (left < 0):
        wheels_commands += 0x80
    if (right < 0):
        wheels_commands += 0x08  

def arm_pub_function(joy_topic):
    global arm_commands
    arm_commands = int(0xFF)

    # Conversion control base rotation
    if (joy_topic.buttons[2] != 0):
        arm_commands ^= (1 << (2*3))
    elif (joy_topic.buttons[1] != 0):
        # arm_commands ^= (0 << (2*3))
        pass
    else:
        arm_commands ^= (3 << (2*3))
    
    # Conversion control upper arm extention
    if (joy_topic.axes[5] <= 0):
        arm_commands ^= (1 << (2*2))
    elif (joy_topic.buttons[5] != 0):
        # arm_commands ^= (0 << (2*2))
        pass
    else:
        arm_commands ^= (3 << (2*2))

    # Conversion control forearm extention
    if (joy_topic.axes[2] <= 0):
        arm_commands ^= (1 << (2*1))
    elif (joy_topic.buttons[4] != 0):
        # arm_commands ^= (0 << (2*1)
        pass
    else:
        arm_commands ^= (3 << (2*1))

    # Conversion control wrist rotation
    if (joy_topic.axes[6] > 0.25):
        arm_commands ^= (1 << (2*0))
    elif (joy_topic.axes[6] < -0.25):
        # arm_commands ^= (0 << (2*0))
        pass
    else:
        arm_commands ^= (3 << (2*0))
    
    #arm_pub.publish(arm_commands)

def callback(data):
    global pub
    #hello_str=data.buttons[0]
    arm_pub_function(data)
    wheels_pub_function(data)
    pub.publish(wheels_commands)
    arm_pub.publish(arm_commands)
    wheels_pub.publish(wheels_commands)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("/joy", Joy, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        timer_pub.publish(Empty())
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()




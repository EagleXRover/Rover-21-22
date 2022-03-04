#!/usr/bin/env python
from scipy import True_
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8

class Autonmy:
    def __init__(self):
        rospy.init_node('autonomy', anonymous=True)
        #Paramathers
        self.maxVelocityPorcentaje = rospy.get_param("/maxVelocityPorcentaje",0.5)
        self.maxVelocity=63
        self.maxDistanceToMaxVelocity = rospy.get_param("/maxDistanceToMaxVelocity",0.5)
        self.maxVelocityDif = rospy.get_param("/maxVelocityDif",.75)
        self.maxAngleToMaxVelocityDif = rospy.get_param("/maxAngleToMaxVelocityDif",.75)
        #Baners
        #Subscribers
        self.arucoSignal="Empty"
        rospy.Subscriber("/aruco/direction", String, self.callbackAruco)
        self.arucoMaxVelocity=30
        self.stopFlag = False
        #Publisher
        self.lVelocity=0
        self.rVelocity=0
        self.pubVelocity=0
        self.pub = rospy.Publisher('/wheels/drivers', UInt16, queue_size=10)
        #RGB
        self.pubRGB = rospy.Publisher('topic_notificationRGB', UInt8, queue_size=10)
        self.red    =bin(0b00000100)
        self.blue   =bin(0b00000010)
        self.green  =bin(0b00000001)

        rospy.spin()

    def callbackAruco (self,data):
        self.arucoSignal=(data.data)
        self.automyNextMove()
    def automyNextMove(self):
        obstaculDetected = False
        if (obstaculDetected):
            self.lVelocity = 64
            self.rVelocity = 64
        else:
            if(self.arucoSignal!='Empty'):
                #Aruco Detected
                self.arucoControl()
            else:
                #Gps
                pass
            self.sendVelocity()
    def arucoControl(self):
        if(self.arucoSignal=='Stop'):
            self.lVelocity = 64
            self.rVelocity = 64
            self.stopFlag = True
        elif(self.arucoSignal=='Forward'):
            self.lVelocity = self.arucoMaxVelocity*self.maxVelocityPorcentaje +64
            self.rVelocity = self.arucoMaxVelocity*self.maxVelocityPorcentaje +64
        elif(self.arucoSignal=='Left'):
            self.lVelocity = self.arucoMaxVelocity*self.maxVelocityPorcentaje*self.maxVelocityDif +64
            self.rVelocity = self.arucoMaxVelocity*self.maxVelocityPorcentaje                     +64
        elif(self.arucoSignal=='Right'):
            self.lVelocity = self.arucoMaxVelocity*self.maxVelocityPorcentaje                     +64
            self.rVelocity = self.arucoMaxVelocity*self.maxVelocityPorcentaje*self.maxVelocityDif +64 

    def sendVelocity(self):
        self.lVelocity=int(self.lVelocity)
        self.rVelocity=int(self.rVelocity)
        self.pubVelocity=(self.lVelocity<<8)+self.rVelocity
        rospy.loginfo("left:"+hex(self.lVelocity)+','+str(self.lVelocity)+"  right"+hex(self.rVelocity)+','+str(self.rVelocity)+"  pub"+hex(self.pubVelocity))
        self.pub.publish(self.pubVelocity)
        if (self.stopFlag):
            self.pub.publish(self.green)
        else:
            self.pub.publish(self.red)
if __name__ == '__main__':
    autonmy=Autonmy()
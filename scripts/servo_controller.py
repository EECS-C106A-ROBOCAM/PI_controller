#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray

class controller(object):
    
    def __init__(self):
        rospy.loginfo("Servo Controller: Initializing")
        rospy.Subscriber("/servo_command", UInt16MultiArray, self.servo_command)

        self.arduino_command = rospy.Publisher("/arduino_command", UInt16MultiArray, queue_size = 10)

    def servo_command(self, msg):
        print(msg)
        
        self.arduino_command.publish(msg)

def main():
    rospy.init_node('servo_controller', anonymous=True)
    servo_controller = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")   

if __name__ == "__main__":
    main()

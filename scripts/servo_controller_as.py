#!/usr/bin/env python

import rospy
import actionlib

from PI_controller.msg import arduino_controlAction, arduino_controlFeedback, arduino_controlGoal, arduino_controlResult
from std_msgs.msg import UInt16MultiArray

class ServoControlAction(object):
    
    _feedback = arduino_controlFeedback()
    _result = arduino_controlResult()
    _arduino_msg = UInt16MultiArray()
    _delay_hz = 8
    _angle_inc = 2

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, arduino_controlAction, execute_cb = self.execute_cb, auto_start=False)
        self._as.start()
        self._feedback.arduino_angles.data = [0, 0, 0, 0, 0,]
        # Publisher
        self._pub = rospy.Publisher("/arduino_command", UInt16MultiArray, queue_size=10)
        self.init_angles()

    # set initial angles
    def init_angles(self):
        self._arduino_msg.data = [0, 0, 0, 0, 0]
        rospy.sleep(1)
        print("setting initial angle to 0")
        self._pub.publish(self._arduino_msg)
        

    def execute_cb(self, goal):

        r = rospy.Rate(self._delay_hz)
        success = True

        angle_steps = [0, 0, 0, 0, 0]
        max_iteration = 0

        to_break = False

        for i in range(len(goal.goal_angles.data)):
            print("servo", i)
            
            # set step as desired angle increment
            step = self._angle_inc if goal.goal_angles.data[i] > self._feedback.arduino_angles.data[i]+self._angle_inc else -self._angle_inc
            iterations = range(self._feedback.arduino_angles.data[i]+step, goal.goal_angles.data[i]+step/abs(step), step)
            
            max_iteration = len(iterations) if len(iterations) > max_iteration else max_iteration
            angle_steps[i] = iterations


        for i in range(max_iteration):

            if to_break:
                break

            for servo in range(len(goal.goal_angles.data)):
               # check if preempt
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    to_break = True
                    break

                if i < len(angle_steps[servo]):
                    angle = angle_steps[servo][i]
                    self._feedback.arduino_angles.data[servo] = angle 
                    self._arduino_msg.data[servo] = angle
                    self._pub.publish(self._arduino_msg)

                    # publish the feedback
                    self._as.publish_feedback(self._feedback)

                r.sleep()

        if success:
            self._result.reached_goal = True
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('arduino_as')
    server = ServoControlAction(rospy.get_name())
    rospy.spin()
            

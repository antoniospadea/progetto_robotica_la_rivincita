#!/usr/bin/env python3

import rospy
import time
import smach
import smach_ros
from std_msgs.msg import Bool, String

# Global Variables
counter_change = 0
picked = False
target = False
stop = False
start = False
pub = rospy.Publisher('/status', String, queue_size=1000)

# Define state WAIT
class WAIT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['home_pose', 'wait'])

    def execute(self, userdata):
        global start, stop
        pub.publish('Wait')
        time.sleep(2)
        if start and not stop:
            return 'home_pose'
        else:
            return 'wait'

# Define state GOTO
class GOTO(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'place', 'pick', 'stop'])

    def execute(self, userdata):
        global target, picked, stop, start
        time.sleep(2)
        start = False
        pub.publish('GoTo')
        if stop:
            return 'stop'
        elif not target:
            return 'wait'
        elif not picked:
            return 'pick'
        else:
            return 'place'

# Define state PICK
class PICK(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'target_position', 'stop'])

    def execute(self, userdata):
        global picked, target, stop
        time.sleep(2)
        pub.publish('Pick')
        if stop:
            return 'stop'
        elif picked:
            return 'target_position'
        else:
            return 'wait'

# Define state PLACE
class PLACE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'home_position', 'stop'])

    def execute(self, userdata):
        global picked, target, stop
        pub.publish('Place')
        time.sleep(2)
        if stop:
            return 'stop'
        elif picked:
            target = False
            picked = False
            return 'home_position'
        else:
            return 'wait'

# ROS Node
def Callback_start(data):
    global start
    rospy.loginfo("I heard %s", data.data)
    start = data.data

def Callback_stop(data):
    global stop
    rospy.loginfo("I heard %s", data.data)
    stop = data.data

def Callback_picked(data):
    global picked
    rospy.loginfo("I heard %s", data.data)
    picked = data.data

def Callback_target(data):
    global target
    rospy.loginfo("I heard %s", data.data)
    target = data.data

def State_Machine():
    rospy.init_node('state_listener', anonymous=True)
    rospy.Subscriber("start_topic", Bool, Callback_start)
    rospy.Subscriber("stop_topic", Bool, Callback_stop)
    rospy.Subscriber("picked_topic", Bool, Callback_picked)
    rospy.Subscriber("target_topic", Bool, Callback_target)

def main():
    State_Machine()
    sm = smach.StateMachine(outcomes=[])
    with sm:
        smach.StateMachine.add('WAIT', WAIT(), transitions={'wait': 'WAIT', 'home_pose': 'GOTO'})
        smach.StateMachine.add('GOTO', GOTO(), transitions={'stop': 'WAIT', 'wait': 'GOTO', 'pick': 'PICK', 'place': 'PLACE'})
        smach.StateMachine.add('PLACE', PLACE(), transitions={'stop': 'WAIT', 'wait': 'PLACE', 'home_position': 'GOTO'})
        smach.StateMachine.add('PICK', PICK(), transitions={'stop': 'WAIT', 'wait': 'PICK', 'target_position': 'GOTO'})
    sis = smach_ros.IntrospectionServer('server_name', sm, '/Pick_and_Place_FSM')
    sis.start()
    outcome = sm.execute()

if __name__ == '__main__':
    main()

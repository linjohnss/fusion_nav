#!/usr/bin/env python3
# license removed for brevity

from asyncio import sleep
from tracemalloc import start
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

import time

pub = rospy.Publisher('amm', String, queue_size=10)

def start_nav(x, y, w):
    global pub
    # print('start nav')
    rospy.loginfo('start nav')
    # pub.publish('Y')
    try:
        # rospy.init_node('movebase_client_py')
        result = movebase_client(x, y, w)           
        if result:
            rospy.loginfo("Goal execution done!")
            pub.publish('Y')
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    return

def movebase_client(x, y, w):
    print('---- move_base ----')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def callback(data):
    rospy.loginfo(data)
    global pub
    # pub.publish('Y')
    
    try:
        result = movebase_client(-0.15, 2.0)
        if result:
            rospy.loginfo("Goal execution done!")            
            pub.publish('Y')
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    
def listener():
    rospy.Subscriber("Intel", String, callback)
    rospy.spin()

if __name__ == '__main__': 
    rospy.init_node('amm_talker', anonymous=True)
    
    time.sleep(2)
    start_nav(0, 0, 0)
    time.sleep(2)
    

    start_nav(1.0, 0, 0)
    listener()
    

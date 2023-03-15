#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from src.util.priority_queue import TaskPriorityQueue
from src.util.priority_manager import TaskPriorityManager

task_priority_queue = TaskPriorityQueue()
task_priority_manager = TaskPriorityManager()

if __name__ == '__main__':
    rospy.init_node('task_manager')

    # Create a publisher to publish messages to the 'task_manager' topic
    pub = rospy.Publisher('task_manager', String, queue_size=10)

    # Publish a message to the 'task_manager' topic every 1 second
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        message = 'Hello, world!'
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from ..srv import AddTask, AddTaskResponse
from util.priority_queue import TaskPriorityQueue
from util.priority_manager import TaskPriorityManager

class TaskManager:
    def __init__(self, ros=False, ros_rate = 1):
        self.task_priority_queue = TaskPriorityQueue()
        self.task_priority_manager = TaskPriorityManager()
        self.ros = ros
        if (self.ros):
            self.startRosnode()
            self.rate = rospy.Rate(ros_rate)
            self.hello_pub = self.startHelloworldPublisher()
            self.addTasksrv = self.startAddTaskService()
        
    def startLoggingService(self):
        print('implement this')

    def startRosnode(self):
        rospy.init_node('task_manager')

    def startHelloworldPublisher(self):
        return rospy.Publisher('task_manager', String, queue_size=10)

    def startAddTaskService(self):
        return rospy.Service('add_task', AddTask, self.addTask)

    def addTask(self, req):
        print('adding new task') # TODO replace with database logger
        response = AddTaskResponse(req)
        return response

if __name__ == '__main__':
    taskmanager = TaskManager(True)

    if (taskmanager.ros):
        while not rospy.is_shutdown():
            message = 'Hello, world!'
            rospy.loginfo(message)
            taskmanager.hello_pub.publish(message)
            taskmanager.rate.sleep()
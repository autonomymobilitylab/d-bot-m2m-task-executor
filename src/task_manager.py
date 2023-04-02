#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from d_bot_m2m_task_executor.srv import AddTask, AddTaskResponse
from d_bot_m2m_task_executor.srv import TaskCall, TaskCallResponse
from util.priority_queue import TaskPriorityQueue
from util.priority_manager import TaskPriorityManager
from definitions.etask import ETask

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

    def do_next_task(self):
        task = self.task_priority_queue.get_task()
        print('Doing task') # TODO replace with database logger
        print(task.stringify_task()) # TODO replace with database logger

        if (task == ETask.STATUS_ELEVATOR):
            # TODO
            resp = self.getElevatorStatus(task)
            print(resp)
        elif (task == ETask.STATUS_CRANE):
            # TODO
            pass
        else:
            # TODO implement functionality
            print('no such task defined') # TODO replace with database logger
            return
        
        print('task execution finished') # TODO replace with database logger

    def getElevatorStatus(self, task):
        rospy.wait_for_service('/elevator_communication/status')
        service_proxy = rospy.ServiceProxy('/elevator_communication/status', TaskCall)
        request = task.task_type
        return service_proxy(request)

if __name__ == '__main__':
    taskmanager = TaskManager(True)
    taskmanager.startLoggingService()

    if (taskmanager.ros):
        while not rospy.is_shutdown():
            message = 'Hello, world!'
            rospy.loginfo(message)
            taskmanager.hello_pub.publish(message)
            taskmanager.do_next_task()
            taskmanager.rate.sleep()
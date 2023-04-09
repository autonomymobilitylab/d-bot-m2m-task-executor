#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from dotenv import dotenv_values
import json

from api.task import Task
from d_bot_m2m_task_executor.srv import AddTask, AddTaskResponse
from d_bot_m2m_task_executor.srv import TaskCall, TaskCallResponse
from util.priority_queue import TaskPriorityQueue
from util.priority_manager import TaskPriorityManager
from definitions.etask import ETask
from db.logger import Logger
from db.postgres_connector import PostgresConnector
from resources.config_loader import ConfigLoader

class TaskManager:
    def __init__(self, config, ros=False):
        self.task_priority_queue = TaskPriorityQueue()
        self.task_priority_manager = TaskPriorityManager()
        self.ros = ros
        if (self.ros):
            self.startRosnode()
            self.rate = rospy.Rate(int(config['TASK_ROS_RATE']))
            self.hello_pub = self.startHelloworldPublisher()
            self.addTasksrv = self.startAddTaskService()
        self.logger = Logger(PostgresConnector(config['DATABASE_NAME'], config['DATABASE_USER'], config['DATABASE_PASSWORD']))
        # update regulary via get_curr_location
        self.location = { "x": 0, "y": 0, "z": 0}
        
    def startLoggingService(self):
        print('implement this')

    def start_location_logging_subscriber(self):
        # TODO
        print('implement this')
        return None
    
    def get_curr_location(self):
        # TODO subscribe to location and save to self.location
        return None

    def startRosnode(self):
        rospy.init_node('task_manager')

    def startHelloworldPublisher(self):
        return rospy.Publisher('task_manager', String, queue_size=10)

    def startAddTaskService(self):
        return rospy.Service('add_task', AddTask, self.add_task_req_handler)

    def add_task_req_handler(self, req):
        print('adding new task') # TODO replace with database logger
        rospy.loginfo(req)
        response = AddTaskResponse(req)
        return response

    def add_task(self, task:Task):
        print('adding new task') # TODO replace with database logger
        rospy.loginfo(task.stringify_task())
        self.task_priority_queue.add_task(task.priority, task)

    def getElevatorStatus(self, task):
        rospy.wait_for_service('/elevator_communication/status')
        service_proxy = rospy.ServiceProxy('/elevator_communication/status', TaskCall)
        request = task.task_type
        return service_proxy(request)

    def get_crane_position(self, task:Task):
        request = task.jsonify()
        rospy.loginfo(request)
        rospy.wait_for_service('/crane_communication/position')
        service_proxy = rospy.ServiceProxy('/crane_communication/position', TaskCall)
        return service_proxy(request)

    def log_location(self):
        self.logger.log_location(self.location)

    def add_task_to_queue(self, task: Task):
        self.task_priority_queue.add_task(task.priority, task)

    def start_task_execution(self):
        task = self.task_priority_queue.get_task()
        self.execute_task(task)


    def execute_task(self, task):
        if (task.task_type == ETask.STOP):
            # TODO
            print("implement this")
        if (task.task_type == ETask.WAIT):
            # TODO
            print("implement this")
        if (task.task_type == ETask.STATUS_ELEVATOR):
            # TODO
            print("implement this")
        if (task.task_type == ETask.STATUS_CRANE):
            # TODO
            print("implement this")
        if (task.task_type == ETask.NAVIGATE):
            # TODO
            print("implement this")
        if (task.task_type == ETask.CALL_ELEVATOR):
            # TODO
            print("implement this")
        if (task.task_type == ETask.POSITION_CRANE):
            res = self.get_crane_position(task)
            rospy.loginfo(res)
        return True

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['DATABASE_NAME', 'DATABASE_USER', 'DATABASE_PASSWORD', 'TASK_ROS_RATE'])
    taskmanager = TaskManager(config, True)
    taskmanager.startLoggingService()
    # s = rospy.Service('add_task', task_manager, add_task_to_queue)
    
    # for testing
    task = Task(ETask.POSITION_CRANE, taskmanager.task_priority_manager.get_priority(ETask.POSITION_CRANE))
    taskmanager.add_task_to_queue(task)

    if (taskmanager.ros):
        while not rospy.is_shutdown():
            # for testing
            taskmanager.add_task_to_queue(task)
            message = 'Hello, world!'
            # rospy.loginfo(message)
            taskmanager.hello_pub.publish(message)
            taskmanager.start_task_execution()
            taskmanager.log_location()
            taskmanager.rate.sleep()

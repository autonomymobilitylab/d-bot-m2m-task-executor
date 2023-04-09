#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# from hdl_localization.msg import Status
from dotenv import dotenv_values
import json

from api.task import Task
from d_bot_m2m_task_executor.srv import AddTask, AddTaskResponse
from d_bot_m2m_task_executor.srv import TaskCall, TaskCallResponse, TaskCallRequest
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

    def start_location_logging_subscriber(self):
        rospy.Subscriber("/odom", Odometry, self.get_curr_location)
    
    def get_curr_location(self, data):
        try:
            rospy.loginfo("I heard %s",data)
            location = {
                "x": data.pose.pose.position.x,
                "y": data.pose.pose.position.y,
                "z": data.pose.pose.position.z
            }
            orientation = {
                "x": data.pose.pose.orientation.x,
                "y": data.pose.pose.orientation.y,
                "z": data.pose.pose.orientation.z,
                "w": data.pose.pose.orientation.w
            }
            self.location = location
        except:
            rospy.loginfo("Failed to extract position")
        return None
    
    #def get_curr_status(self, data):
    #    try:
    #        rospy.loginfo(data.pose.position.x)
    #    except:
    #        rospy.loginfo("Failed to extract position")
    #    return None

    #def start_status_logging_subscriber(self):
    #    rospy.Subscriber("/status", Status, self.get_curr_status)

    def startRosnode(self):
        rospy.init_node('task_manager')

    def startHelloworldPublisher(self):
        return rospy.Publisher('task_manager', String, queue_size=10)

    def startAddTaskService(self):
        return rospy.Service('/task_manager/add_task', TaskCall, self.add_task_req_handler)

    def add_task_req_handler(self, req):
        task = Task().load(req.task)
        rospy.loginfo(task)
        try:
            self.add_task(task)
            rospy.loginfo("new task created")
        except:
            task.error = "failed to create task"
            task.success = False
            rospy.loginfo(task.error)
        task_json = task.jsonify()
        response = TaskCallResponse()
        response.task = task_json
        return response

    def add_task(self, task:Task):
        print('adding new task') # TODO replace with database logger
        self.task_priority_queue.add_task(task.priority, task)

    def getElevatorStatus(self, task):
        rospy.wait_for_service('/elevator_communication/status')
        service_proxy = rospy.ServiceProxy('/elevator_communication/status', TaskCall)
        request = task.task_type
        return service_proxy(request)

    def get_crane_position_client(self, task:Task):
        task_json = task.jsonify()
        rospy.wait_for_service('/crane_communication/position')
        service_proxy = rospy.ServiceProxy('/crane_communication/position', TaskCall)
        request = TaskCallRequest()
        request.task = task_json
        return service_proxy(request)

    def get_crane_status_client(self, task:Task):
        task_json = task.jsonify()
        rospy.wait_for_service('/crane_communication/status')
        service_proxy = rospy.ServiceProxy('/crane_communication/status', TaskCall)
        request = TaskCallRequest()
        request.task = task_json
        return service_proxy(request)

    def get_crane_stop_client(self, task:Task):
        task_json = task.jsonify()
        rospy.wait_for_service('/crane_communication/stop')
        service_proxy = rospy.ServiceProxy('/crane_communication/stop', TaskCall)
        request = TaskCallRequest()
        request.task = task_json
        return service_proxy(request)
    
    def get_area_protection_client(self, task:Task):
        task_json = task.jsonify()
        rospy.wait_for_service('/beacon_communication/protection')
        service_proxy = rospy.ServiceProxy('/beacon_communication/protection', TaskCall)
        request = TaskCallRequest()
        request.task = task_json
        return service_proxy(request)

    def log_location(self):
        log_res = self.logger.log_location(self.location)
        if (log_res == False):
            rospy.loginfo("Logging of location failed")

    def add_task_to_queue(self, task: Task):
        self.task_priority_queue.add_task(task.priority, task)

    def start_task_execution(self):
        task = self.task_priority_queue.get_task()
        self.execute_task(task)

    def execute_task(self, task:Task):
        # TODO make most task executions asyncronous
        # TODO reduce complexity, make neater
        log_res = None
        if (task):
            log_res = self.logger.logAction(task)
            if (log_res == False):
                rospy.loginfo("Logging of action failed")
            rospy.loginfo(task.jsonify())
        else:
            return
        if (task.task_type == ETask.STOP):
            # TODO
            print("implement this")
        elif (task.task_type == ETask.WAIT):
            # TODO
            print("implement this")
        elif (task.task_type == ETask.STATUS_ELEVATOR):
            # TODO
            print("implement this")
        elif (task.task_type == ETask.STATUS_CRANE):
            res = self.get_crane_status_client(task)
            task_res = Task().load(res.task)
            if (task_res.error == None and task_res.success == True):
                rospy.loginfo("Crane is Stopped")
            else:
                # TODO suspend current operation
                rospy.loginfo("Crane is moving")
        elif (task.task_type == ETask.NAVIGATE):
            # TODO
            print("implement this")
        elif (task.task_type == ETask.CALL_ELEVATOR):
            # TODO
            print("implement this")
        elif (task.task_type == ETask.POSITION_CRANE):
            res = self.get_crane_position_client(task)
            task_res = Task().load(res.task)
            if task_res.success == False:
                rospy.loginfo("Crane position check failed")
        elif (task.task_type == ETask.STOP_CRANE):
            res = self.get_crane_stop_client(task)
            task_res = Task().load(res.task)
            if task_res.success == False:
                rospy.loginfo("Crane stopping failed")
                # TODO what to do after crane stop failed
        elif (task.task_type == ETask.START_WORK_AREA_PROTECTION):
            res = self.get_area_protection_client(task)
            task_res = Task().load(res.task)
            if task_res.success == False:
                rospy.loginfo("Beacon workarea protection failed")
                # TODO what to do after crane stop failed
        else:
            rospy.loginfo("Task Type not initialized")
        return True

if __name__ == '__main__':
    config = dotenv_values("resources/.env")
    if bool(config) == False:
        config = ConfigLoader()
        config.load(['DATABASE_NAME', 'DATABASE_USER', 'DATABASE_PASSWORD', 'TASK_ROS_RATE'])
    taskmanager = TaskManager(config, True)
    taskmanager.start_location_logging_subscriber()

    new_task = Task(ETask.START_WORK_AREA_PROTECTION, taskmanager.task_priority_manager.get_priority(ETask.START_WORK_AREA_PROTECTION))
    new_task.device_id = 106
    taskmanager.add_task_to_queue(new_task)

    if (taskmanager.ros):
        while not rospy.is_shutdown():
            # for testing
            # taskmanager.add_task_to_queue(task)
            taskmanager.hello_pub.publish("Hello World")
            taskmanager.start_task_execution()
            taskmanager.log_location()
            taskmanager.rate.sleep()

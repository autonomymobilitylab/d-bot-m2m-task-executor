import rospy
from std_msgs.msg import String

from src.util.priority_queue import TaskPriorityQueue
from src.util.priority_manager import TaskPriorityManager
from src.api.task import Task
from src.enum.etask import ETask

task_priority_queue = TaskPriorityQueue()
task_priority_manager = TaskPriorityManager()

def add_task_to_queue(task_id):
    task = Task(task_id,task_priority_manager.get_priority(task_id))
    task_priority_queue.add_task(task)

def start_task_execution(task):
    task = task_priority_queue.get_task()
    execute_task(task)
    
def execute_task(task):
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
    return True

if __name__ == '__main__':
    rospy.init_node('task_manager')
    s = rospy.Service('add_task', task_manager, add_task_to_queue)
    rospy.spin()

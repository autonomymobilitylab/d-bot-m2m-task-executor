import rospy
from std_msgs.msg import String

from src.util.priority_queue import TaskPriorityQueue
from src.util.priority_manager import TaskPriorityManager
from src.api.task import Task

task_priority_queue = TaskPriorityQueue()
task_priority_manager = TaskPriorityManager()

def add_task_to_queue(task_id):
    task = Task(task_id,task_priority_manager.get_priority(task_id))
    task_priority_queue.add_task(task)

if __name__ == '__main__':
    rospy.init_node('task_manager')
    s = rospy.Service('add_task', task_manager, add_task_to_queue)
    rospy.spin()

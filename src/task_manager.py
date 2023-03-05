import rospy
from std_msgs.msg import String

from src.util.priority_queue import TaskPriorityQueue
from src.util.priority_manager import TaskPriorityManager

task_priority_queue = TaskPriorityQueue()
task_priority_manager = TaskPriorityManager()

if __name__ == '__main__':
    rospy.init_node('task_manager')

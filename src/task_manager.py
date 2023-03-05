import rospy
from std_msgs.msg import String

from src.priority_queue import TaskPriorityQueue

task_priority_queue = TaskPriorityQueue()

if __name__ == '__main__':
    rospy.init_node('task_manager')

# import library
import unittest
import json

from api.task import Task
from util.priority_manager import TaskPriorityManager
from util.priority_queue import TaskPriorityQueue
from definitions.etask import ETask
from definitions.etask_priority import ETaskPriority

class UnitTests(unittest.TestCase):

    def test_get_priority_high(self):
        task_type = ETask.STOP
        res = TaskPriorityManager().get_priority(task_type)
        self.assertEqual(res, ETaskPriority.HIGH.value)

    def test_get_priority_vlow(self):
        task_type = ETask.CHECK_AUTH_KONE
        res = TaskPriorityManager().get_priority(task_type)
        self.assertEqual(res, ETaskPriority.VERY_LOW.value)

    def test_add_task(self):
        queue = TaskPriorityQueue()
        task_type = ETask.POSITION_CRANE
        task = Task(task_type, TaskPriorityManager().get_priority(task_type))
        queue.add_task(task.priority,task)
        res = queue.get_task()
        self.assertEqual(res.task_type, task.task_type, "Result from get_Task should be same as added task")
        self.assertEqual(res.priority, task.priority, "Result from get_Task should be same as added task")
    
    def test_load_task_from_string(self):
        failure_msg = "Result from string should be same as created object"
        task_type = ETask.POSITION_CRANE
        task = Task(task_type, TaskPriorityManager().get_priority(task_type))
        task.location = {
            "x": 1,
            "y": 2,
            "z": 3
        }
        task_json = task.jsonify()
        print(task_json)
        new_task = Task().load(task_json)
        self.assertEqual(new_task.task_type, task.task_type, failure_msg)
        self.assertEqual(new_task.priority, task.priority, failure_msg)
        self.assertEqual(new_task.success, task.success, failure_msg)
        self.assertEqual(new_task.location, task.location, failure_msg)

if __name__ == '__main__':
    unittest.main()
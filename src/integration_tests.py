import unittest

from api.task import Task
from util.priority_manager import TaskPriorityManager
from util.priority_queue import TaskPriorityQueue
from definitions.etask import ETask
from definitions.etask_priority import ETaskPriority
from db.logger import Logger
from db.postgres_connector import PostgresConnector
from db.localizationDAO import LocalizationDAO
from localization.localizer import Localizer

class UnitTests(unittest.TestCase):
    
    def test_action_logging(self):
        task_type = ETask.STOP
        task = Task(task_type, TaskPriorityManager().get_priority(task_type))
        logger = Logger(PostgresConnector("mydatabase", "myuser", "mypassword"))
        success = logger.logAction(task)
        self.assertEqual(success, True, "Should be true if logging was successful")

    def test_location_logging(self):
        logger = Logger(PostgresConnector("mydatabase", "myuser", "mypassword"))
        success = logger.log_location({"x":1, "y": 2, "z":3})
        self.assertEqual(success, True, "Should be true if logging was successful")

    def test_localization(self):
        local = Localizer(LocalizationDAO(PostgresConnector("mydatabase", "myuser", "mypassword")))
        location = {
            "x": 100,
            "y": 100,
            "z": 100
        }
        coords = local.localize(1, location)
        print(coords)
        self.assertNotEqual(coords['x'], location['x'], "Should be different from original values")
        self.assertNotEqual(coords['y'], location['y'], "Should be different from original values")
        self.assertNotEqual(coords['z'], location['z'], "Should be different from original values")

if __name__ == '__main__':
    unittest.main()
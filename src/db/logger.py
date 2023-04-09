import json

from db.postgres_connector import PostgresConnector
from api.task import Task

class Logger:
    def __init__(self, db:PostgresConnector):
        self.db = db

    def log_location(self, location):
        location = json.dumps(location)
        query = f"INSERT INTO public.dbot_location_log(bot_id, info, bot_location) VALUES(1, 'task_executor_logging', '{location}'::jsonb);"
        self.db.connect()
        result = self.db.execute(query)
        self.db.disconnect()
        return result

    def logAction(self, task: Task, ID=None):
        if (id != None):
            # TODO modify logged action in case of error or completion
            print('implement action log modification')
        self.db.connect()
        query = f"INSERT INTO public.dbot_action_log(bot_id, info, action_id) VALUES(1, '{task.jsonify()}', {task.task_type});"
        result = self.db.execute(query)
        self.db.disconnect()
        return result

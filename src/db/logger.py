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
        location = json.dumps(task.location)
        self.db.connect()
        query = f"INSERT INTO public.dbot_action_log(bot_id, info, action_id, bot_location) VALUES(1, '{task.jsonify()}', {task.task_type}, '{location}'::jsonb);"
        result = self.db.execute(query)
        self.db.disconnect()
        return result

    def log_action_update(self, task: Task, ID=None):
        self.db.connect()
        location = json.dumps(task.location)
        query = f"Update public.dbot_action_log set info ='{task.jsonify()}', error_msg ='{task.error}', bot_location='{location}'::jsonb where action_id = {task.task_type} and log_timestamp=(SELECT MAX(log_timestamp) FROM public.dbot_action_log T2 WHERE T2.action_id = {task.task_type});"
        result = self.db.execute(query)
        self.db.disconnect()
        return result
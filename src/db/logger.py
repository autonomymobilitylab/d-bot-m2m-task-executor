from db.postgres_connector import PostgresConnector

class Logger:
    def __init__(self, db:PostgresConnector):
        self.db = db

    def log_location(self, location):
        query = f"INSERT INTO public.dbot_location_log(bot_id, info, bot_location) VALUES(1, 'task_executor_logging', '{location}'::jsonb);"
        self.db.connect()
        self.db.execute(query)
        self.db.disconnect()
        return None

    def logAction(self, task, ID=None):
        if (id != None):
            # TODO modify logged action in case of error or completion
            print('implement action log modification')
        self.db.connect()
        query = f"INSERT INTO public.dbot_action_log(bot_id, info, action_id) VALUES(1, '{task.stringify_task()}', {task.task_type});"
        self.db.execute(query)
        self.db.disconnect()
        return None

from db.postgres_connector import PostgresConnector

class Logger:
    def __init__(self, db:PostgresConnector):
        self.db = db

    def log_location(self, location):
        query = f"INSERT INTO public.dbot_location_log VALUES(1, 'task_executor_logging', '', '{location}'::jsonb);"
        self.db.connect()
        self.db.execute(query)
        self.db.disconnect()
        return None

    def logAction(self, ID=None):
        if (id != None):
            # TODO Update existing action log row in db
            # for example error or success
            print('Implement this')
            return None
		# TODO insert now action log row in db
        print('Implement this')
        return None

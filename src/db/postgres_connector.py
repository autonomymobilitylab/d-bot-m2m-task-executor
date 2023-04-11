import psycopg2

class PostgresConnector:
    def __init__(self, dbname, user, password, host='localhost', port=5432, ros = None):
        self.dbname = dbname
        self.user = user
        self.password = password
        self.host = host
        self.port = port
        self.conn = None
        self.cursor = None
        self.ros = ros
    
    def connect(self):
        try:
            self.conn = psycopg2.connect(
                dbname=self.dbname,
                user=self.user,
                password=self.password,
                host=self.host,
                port=self.port
            )
            self.cursor = self.conn.cursor()
            print(f"Connected to database {self.dbname} successfully!")
        except Exception as e:
            if (self.ros):
                self.ros.loginfo(f"connect to database failed {e}")
            print(f"Unable to connect to database: {e}")
    
    def execute(self, query):
        try:
            self.cursor.execute(query)
            self.conn.commit()
            print("Query executed successfully!")
            return True
        except Exception as e:
            if (self.ros):
                self.ros.loginfo(f"Unable to execute query: {e}")
            print(f"Unable to execute query: {e}")
            return False

    def fetch(self, query):
        try:
            self.connect()
            self.cursor.execute(query)
            # self.conn.commit()
            print("Query executed successfully!")
            res = self.cursor.fetchall()
            self.disconnect()
            return res
        except Exception as e:
            if (self.ros):
                self.ros.loginfo(f"Unable to execute fetch: {e}")
            print(f"Unable to execute query: {e}")
    
    def disconnect(self):
        try:
            self.cursor.close()
            self.conn.close()
            print(f"Disconnected from database {self.dbname} successfully!")
        except Exception as e:
            if (self.ros):
                self.ros.loginfo(f"Unable to execute disconnect: {e}")
            print(f"Unable to disconnect from database: {e}")
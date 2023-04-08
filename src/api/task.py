class Task():
    
	def __init__(self, task_type, priority):
		self.task_type = task_type
		self.priority = priority
	
	def stringify_task(self):
		data_dict = self.__dict__
		return ', '.join([f"{key}='{value}'" for key, value in data_dict.items()])

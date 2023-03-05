from src.enum.etask import ETask
from src.enum.etask_priority import ETaskPriority

class TaskPriorityManager:
    
	def __init__(self):
		self.task_priorities = {
		ETask.NAVIGATE: ETaskPriority.MEDIUM,
		ETask.STOP: ETaskPriority.HIGH,
		ETask.WAIT: ETaskPriority.HIGH,
		ETask.CHECK_AUTH_ILMATAR: ETaskPriority.VERY_LOW,
		ETask.CHECK_AUTH_KONE: ETaskPriority.VERY_LOW,
		ETask.CHECK_AUTH_NOCCELA: ETaskPriority.VERY_LOW,
		ETask.STATUS_ELEVATOR: ETaskPriority.MEDIUM,
		ETask.STATUS_CRANE: ETaskPriority.MEDIUM
	}

	def get_priority(self, task):
		return self.task_priorities[task].value
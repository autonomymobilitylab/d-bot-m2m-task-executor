from definitions.etask import ETask
from definitions.etask_priority import ETaskPriority

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
		ETask.STATUS_CRANE: ETaskPriority.MEDIUM,
		ETask.POSITION_CRANE: ETaskPriority.MEDIUM,
		ETask.STOP_CRANE: ETaskPriority.VERY_HIGH,
		ETask.START_WORK_AREA_PROTECTION: ETaskPriority.VERY_HIGH,
		ETask.TAG_LOCATIONS: ETaskPriority.MEDIUM
	}

	def get_priority(self, task:ETask):
		return self.task_priorities[task].value
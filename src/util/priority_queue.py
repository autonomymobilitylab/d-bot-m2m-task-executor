import heapq

class TaskPriorityQueue:
    def __init__(self):
        self.tasks = []
        # counter is used to solve ties in priority queue
        self.counter = 0
    
    def add_task(self, priority, task):
        heapq.heappush(self.tasks, (priority, self.counter, task))
        self.counter += 1
    
    def get_task(self):
        if len(self.tasks) > 0:
            return heapq.heappop(self.tasks)[2]
        else:
            return None

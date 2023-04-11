# d_bot_m2m_task_executor
d-bot m2m task executor ROS package. Queues tasks based on set priorities and executes them.



# Usage

Create tasks in code or with ros service "/task_manager/add_task"  
Tasks are run based on set ros rate frequency and each task type has a specific code to handle each of the task types.  
If there is database setup task executor can connect to it to log actions, database table:"dbot_action_log", and current location, database table:"dbot_location_log", based on odometry.
## with service

this service call requires String type message with "task"
{\"task_type\": 12, \"priority\": 1, \"device_id\": 106, \"location\": null, \"success\"\
  : false, \"error\": null, \"task_id\": null}  

## with code

Task(ETask.START_WORK_AREA_PROTECTION, taskmanager.task_priority_manager.get_priority(ETask.START_WORK_AREA_PROTECTION))  

this creates task with type "START_WORK_AREA_PROTECTION" and gets priority of the task automaticly.

## 

## Launched 
Lauched as a part of d-bot-m2m-launcher
roslauch d_bot_m2m_task_executor task_manager.py

# Localization

if task requires fetching data from device with it's own coordinate system it needs to be converted in to dbots own coordinates.  

Localization currently needs devices coordinates systems start points location. This needs to be measured with d-bots map made with point cloud. To this device location is added devices own coordinate values in meters to get localized point for interaction.

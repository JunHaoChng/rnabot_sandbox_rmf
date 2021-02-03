# rna task status definition:
string SUCCESS = "Success"
string COMPLETED = "Completed"
string FAILED = "Failed"
string CANCELLED = "Cancelled"
string EXECUTING = "Executing"
string ABORTED = "Aborted"

# the value can be : "Success" , "Completed", "Failed", "Cancelled", "Executing", and "Aborted"
string status

# None empty unique string, which associated with the task  
string task_id

# in format of yyyy-mm-dd hh:mm:ss, i.e.  2019-04-24 16:18:04
string started_time
# in format of yyyy-mm-dd hh:mm:ss, i.e.  2019-04-24 16:28:04
string ended_time

# The details of the task status, 
# i.e. the task_status=’Aborted' and reason in description is 'Task name not defined', if the assigned task is not difined
string description
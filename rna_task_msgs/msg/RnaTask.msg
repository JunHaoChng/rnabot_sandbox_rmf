# rna task definition:
string VSM="VSM"
string MEDICINE_DELIVERY ="MEDICINE_DELIVERY"
string ITEM_DELIVERY ="ITEM_DELIVERY"
string LOAD_ITEM = "LOAD_ITEM"
string CODE_RED_BLUE = "CODE_RED_BLUE"
string CANCEL_TASK = "CANCEL_TASK"
string GO_NURSE_STATION = "GO_NURSE_STATION"
string GO_HOME = "GO_HOME"

# schedule type definition:
string NONE_SCHEDULE ="NONE_SCHEDULE"
string HALF_HOUR ="HALF_HOUR"
string HOURLY ="HOURLY"
string DAILY ="DAILY"

# None empty unique string, i.e. T001
string task_id

# the tas name can be: "VSM", "MEDICINE_DELIVERY", "ITEM_DELIVERY", "LOAD_ITEM", "CODE_RED_BLUE", "CANCEL_TASK", "GO_NURSE_STATION" and "GO_HOME"
string task_name

# in format of yyyy-mm-dd hh:mm:ss, i.e.  2019-04-24 16:18:04, Default=""
string schedule_time

# The value can be : "NONE_SCHEDULE", "HALF_HOUR", "HOURLY" and "DAILY", 
# if the schedule type is NONE_SCHEDULE, the task will be put into execution queue immediately, Default="NONE_SCHEDULE"
string schedule_type

# The priority of the task to be executed, the higher value, the lower priority to be executed, Default=0 
uint32 priority

# None empty unique string, i.e. RNA001
string robot_name

# The patient's name for the task, i.e. "Bill Cheng"
string patient_name

# The registered patient's unique number.
# This is unique patient number, for face recognition face id and  and VSM record purpose. 
uint32 patient_id

# patient's wrist tag QR code value, for authentication purpose
string barcode

# the associated bed number for the registered patient
uint32 bed_id

# predefined patient engage points for navigation
RnaPredefinepos[] pre_def_pos

# the position for robot charging station or home 
Location home_position

# the position for nurse station
Location nurse_station

# the robot holding point during the emergency 
RnaEmergency escape

# specific item name for item delivery task
string item_name

# Load item operation: either "open" or "close", the robot compartment will be opened or closed by specified the operation
string load_operation
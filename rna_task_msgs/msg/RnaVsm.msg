# None empty unique string, i.e. RNA001
string robot_name

# The registered patient's unique number.
uint32 patient_id

# in format of yyyy-mm-dd hh:mm:ss, i.e.  2019-04-24 16:18:04
string record_time 

# heart beat times per minute, i.e. 65
uint32 heart_rate

# blood pressure in format of (high, low) , i.e. (110, 80)
uint32[] blood_pressure

# body temperature in celsius, i.e.  36.5
float32 temperature

# breath times per minute, i.e. 18
uint32 respiration_rate

# blood oxygen level of percentage, i.e. 98
uint32 spo2

# patient pain level, pain score value form 0-10
uint32 pain_score

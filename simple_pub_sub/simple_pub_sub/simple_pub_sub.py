#!/bin/usr/env python3

import argparse, sys
import time, os
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
import rclpy

# from rmf_task_msgs.msg import Loop
from rmf_fleet_msgs.msg import Location as rmf_loc
from rmf_fleet_msgs.msg import FleetState, RobotState, RobotMode
from rmf_fleet_msgs.msg import PathRequest, ModeRequest, DestinationRequest, ModeParameter

from rna_task_msgs.msg import RnaTask, RnaTaskstatus, Location, RnaEmergency, RnaPredefinepos, RnaVsm

# the robot 
ROBOT_UNIQUE_ID = os.environ.get('ROBOT_ID', 'RNAxx')
# topic define: 
RMF_TASK                 = '/rna_task'
RMF_TASK_STATUS          = '/rna_task_status'
RMF_VSM_RECORD           = '/rna_vsm_record'
RMF_PARSE_REQUESTS       = '/parse_requests'
RMF_FLEET_STATES         = '/fleet_states'
RMF_MODE_REQUESTS        = '/mode_requests'
RMF_PATH_REQUESTS        = '/path_requests'
RMF_DESTINATION_REQUESTS = '/destination_requests'

def parse_argv(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-task_id", "--task_id", type=str, help="task id", default='T001')
    parser.add_argument("-task_name", "--task_name", type=str, help="task name", default='VSM')
    parser.add_argument("-name", "--patient_name", type=str, help="patient name", default='John')
    parser.add_argument("-qr", "--qr_code", type=str, help="qr code value", default='1321351')
    parser.add_argument("-item", "--item_name", type=str, help="item name", default='water')
    parser.add_argument("-load", "--load_operation", type=str, help="open, or close compartment", default='open')
    parser.add_argument("-patient_id", "--patient_id", type=int, help="patient_id", default=2)
    parser.add_argument("-robot", "--robot_id", type=str, help="robot id", default=ROBOT_UNIQUE_ID)
    parser.add_argument("-timer", "--enable_timer", type=int, help="enable_timer for debugging", default=1)
    parser.add_argument("-status", "--show_fleet_state", type=int, help="show_fleet_state msg", default=0)
    parser.add_argument("-bed", "--bed", type=int, help="bed number: i.e 1, 2 or...", default=1)
    parser.add_argument("-escape", "--escape_on", type=int, help="1 is on , 0 is off", default=1)
    parser.add_argument("-topic", "--rmf_topic", type=int, help="1=RnaTask, 2=ModeRequest, 3=PathRequest, 4=DestinationRequest", default=1)
    parser.add_argument("-mode", "--mode", type=int, help="robot mode", default=1)
    parser.add_argument("-schedule_type", "--schedule_type", type=str, help="schedule_type", default='NONE_SCHEDULE')
    parser.add_argument("-schedule_time", "--schedule_time", type=str, help="in format of yyyy-mm-dd hh:mm:ss", default='yyyy-mm-dd hh:mm:ss')

    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    
    argv = remove_ros_args(args=argv)
    args = parser.parse_args(argv)
    return args

class Simple_Pub_Sub(Node):
    def __init__(self, rmf_task, enable_timer=True, show_fleet_state=False, rmf_topic=1):
        super().__init__('simple_pub')
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        

        self.rmf_task = rmf_task
        self.enable_timer = enable_timer
        self.show_fleet_state = show_fleet_state
        if rmf_topic ==1:
            self.pub_rmf_task = self.create_publisher(RnaTask, RMF_TASK, qos_profile=qos_reliable) #  send the task to robot
        elif rmf_topic ==2:
            self.pub_rmf_task = self.create_publisher(ModeRequest, RMF_MODE_REQUESTS, qos_profile=qos_reliable)
        elif rmf_topic ==3:
            self.pub_rmf_task = self.create_publisher(PathRequest, RMF_PATH_REQUESTS, qos_profile=qos_reliable)
        else: # rmf_topic ==4:
            self.pub_rmf_task = self.create_publisher(DestinationRequest, RMF_DESTINATION_REQUESTS, qos_profile=qos_reliable)        

        # receive vsm record from Robot
        self.create_subscription(
            RnaVsm, RMF_VSM_RECORD, self.vsm_record_callback, qos_profile=qos_reliable)
        
        # receive task status from Robot
        self.create_subscription(
            RnaTaskstatus, RMF_TASK_STATUS, self.rmf_task_status_callback, qos_profile=qos_reliable)        
        
        # receive FleetState status 
        self.create_subscription(
            FleetState, RMF_FLEET_STATES, self.robot_status_callback, qos_profile=qos_reliable)
        
        if self.enable_timer:
            timer_period = 1.0
            self.tmr = self.create_timer(timer_period, self.timer_callback)
    
    def vsm_record_callback(self, vsm_record):
        print(vsm_record.robot_name, vsm_record.patient_id, vsm_record.record_time, vsm_record.heart_rate)
        print(vsm_record.blood_pressure, vsm_record.temperature, vsm_record.respiration_rate, vsm_record.spo2, vsm_record.pain_score)
    def rmf_task_status_callback(self, rmf_status):
        print(rmf_status.task_id, rmf_status.status, rmf_status.started_time, rmf_status.ended_time, rmf_status.description)
    def robot_status_callback(self, fleet_state):
        if self.show_fleet_state:
            print('fleet name:{}, robot name:{}, model:{}, task_id:{}'.
                                   format(fleet_state.name, fleet_state.robots[0].name, fleet_state.robots[0].model, fleet_state.robots[0].task_id))
            print(fleet_state.robots[0].battery_percent, fleet_state.robots[0].mode.mode)
            print(fleet_state.robots[0].location.t)
            print(fleet_state.robots[0].location.x, fleet_state.robots[0].location.y, fleet_state.robots[0].location.yaw, fleet_state.robots[0].location.level_name)
            #print(fleet_state.robots[0].path[0].t)
    def timer_callback(self):
        print("debug: timer_callbak, topic issued only once")
        self.tmr.cancel()
        self.pub_rmf_task.publish(self.rmf_task)        


def create_rna_task(args):
    # some navigation points hardcode below, to be changed accordingly 
    #                 x         y    heading  
    NURSE_STATION = [0.581, 19.511, -1.511]
    #                 x         y    heading     
    HOME_POSITION = [-2.912, 18.559, 1.561]
    #                 x         y    heading  bed_heading
    Neutral_point = [-2.324, 21.091, 1.557, 1.557]
    #                 x         y    heading  bed_heading
    Bedside_left  = [-3.170, 21.261, 0.9025, 1.557]
    #                 x         y    heading  bed_heading
    Bedside_right = [-1.485, 21.229, 2.143, 1.557]

    home = HOME_POSITION
    PRDEF_POS = (Bedside_left, Neutral_point, Bedside_right)
    
    task = RnaTask()
    # common parameters for all the tasks
    task.task_id =  args.task_id #'T001'
    task.task_name = args.task_name        
    task.robot_name = args.robot_id
    # home position
    loc = Location()
    loc.x = home[0]
    loc.y = home[1]
    loc.heading = home[2]
    task.home_position = loc    

    if task.task_name in ('VSM', 'MEDICINE_DELIVERY', 'ITEM_DELIVERY'):
        task.bed_id = args.bed
        task.patient_name = args.patient_name
        task.patient_id = args.patient_id
        task.barcode = args.qr_code
        task.item_name = args.item_name
        
        # schedule task checking
        task.schedule_type = args.schedule_type
        if task.schedule_type != 'NONE_SCHEDULE': # if it's not NONE_SCHEDULE, the field of task.schedule_time need to be specified
            task.schedule_time = args.schedule_time # in format of "yyyy-mm-dd hh:mm:ss"

        # predefined patient engage points
        for pos in PRDEF_POS:
            prepos = RnaPredefinepos()
            loc = Location()
            loc.x = pos[0]
            loc.y = pos[1]
            loc.heading = pos[2]
            prepos.point=loc
            prepos.bed_heading= pos[3]
            task.pre_def_pos.append(prepos)     
    elif  task.task_name == 'GO_NURSE_STATION':
        loc = Location()
        loc.x = NURSE_STATION[0]
        loc.y = NURSE_STATION[1]
        loc.heading = NURSE_STATION[2]
        task.nurse_station = loc
    elif  task.task_name == 'CODE_RED_BLUE':    
        escape = RnaEmergency()
        loc = Location()
        loc.x = home[0] # hardcode here for temp testing, to be defined the emergency holding point
        loc.y = home[1]
        loc.heading = home[2]
        escape.point = loc
        escape.emergency_on = bool(args.escape_on) # True/False to switch it on/off
        task.escape = escape
    elif  task.task_name == 'LOAD_ITEM':
        task.item_name = args.item_name
        task.load_operation = args.load_operation    
    #elif task.task_name == 'CANCEL_TASK' or task.task_name == 'GO_HOME':        
        #pass
    return task
def create_mode_requests(args):
    task = ModeRequest()
    #task.fleet_name = 'rna'
    #task.robot_name = ROBOT_UNIQUE_ID
    #task.task_id = 'T001'
    #mode = RobotMode()
    #mode.mode = args.mode
    #task.mode = mode
    #para=ModeParameter()
    #para.name = 'tt1'
    #para.value = 'test'
    #task.parameters.append(para)
    return task
def create_path_requests():
    task = PathRequest()
    #task.fleet_name = 'rna'
    #task.robot_name = ROBOT_UNIQUE_ID
    #task.task_id = 'T001'
    #loc = rmf_loc()
    #loc.x = HOME_POSITION[0]
    #loc.y = HOME_POSITION[1]
    #loc.yaw = HOME_POSITION[2]
    #loc.level_name = 'healthcare room'
    #loc.index = 123456
    #task.path.append(loc)
    return task
def create_destination_requests():
    task = DestinationRequest()
    #task.fleet_name = 'rna'
    #task.robot_name = ROBOT_UNIQUE_ID
    #task.task_id = 'T002'
    #loc = rmf_loc()
    #loc.x = HOME_POSITION[0]
    #loc.y = HOME_POSITION[1]
    #loc.yaw = HOME_POSITION[2]
    #loc.level_name = 'healthcare room'
    #loc.index = 78910
    #task.destination = loc
    return task



def main(argv=sys.argv[1:]):
    args = parse_argv()
    rclpy.init(args=argv)

    if args.rmf_topic == 1: # rna_task
        task = create_rna_task(args)    
    elif args.rmf_topic == 2: # mode_requests
        task = create_mode_requests(args)            
    elif args.rmf_topic == 3: # path_requests
        task = create_path_requests()
    elif args.rmf_topic == 4: # destination_requests
        task = create_destination_requests()

    node = Simple_Pub_Sub(task, bool(args.enable_timer), bool(args.show_fleet_state), args.rmf_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

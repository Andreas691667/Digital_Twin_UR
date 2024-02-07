# parse_task parses a task in a csv file to the URScript format
# TODO:
# 1) ---
# __joint_positions_to_signature and __target_pose_to_signature is not implemented correctly yet!
# Current output: movej(3.14,-1.52,0.0001,-1.53589,0.006,0.02391101,0.3,0.6)
# Wanted output: movej([3.14, -1.5196582, 0.0001, -1.53589, 0.006, 0.02391101], a=0.3 , v=0.6)
# This could be corrected by making functions that can add strings to each index e.g. ["0.3"] -> "a=0.3",
# and by making a generic function that can seperate a list by commas and put it inside some wanted brackets, e.g. ["1", "2", "3"] -> "[1, 2, 3]"

import sys
sys.path.append("..")

from config.task_config import TASK_CONFIG
import csv


def parse_task(file_path = "task.csv"):
    # 1) Loop through operations
    # 2) Get argument type
    # 3) Based on argument type construct code line
    task_parsed = "task_function(): \n"
    signatures: list[str] = []
    with open(file_path) as task:
        csvreader = csv.reader(task)
        next(csvreader) # skip header row
        for row in csvreader:
            # Get subtask as list of arguments
            subtask_str = row[0]
            subtask_arr = __str_to_arr(subtask_str)

            # Get argument type and number of arguments
            arg_type = subtask_arr[0]
            print(arg_type)
            arg_num = int(subtask_arr[1])
            args = subtask_arr[2:arg_num+2]
            
            # Define string signature of row
            signature_str = ""

            # Handle based on case
            if (arg_type == TASK_CONFIG.TARGET_POSE):
                signature_str = __target_pose_to_signature(args)
            if (arg_type == TASK_CONFIG.JOINT_POSITIONS):
                signature_str = __joint_positions_to_signature(args)
            if (arg_type == TASK_CONFIG.STOP):
                signature_str = __stop_to_signature(args)
            task_parsed += "\t " + signature_str + "\n"
            
    
    task_parsed += "end \n task_function() \n"
    return task_parsed


def __stop_to_signature(deceleration: list):
    signature_str = __create_comma_seperated_signature(TASK_CONFIG.ur_script_mappings[TASK_CONFIG.STOP], deceleration)
    return signature_str

def __joint_positions_to_signature(joint_positions: list):
    signature_str = __create_comma_seperated_signature(TASK_CONFIG.ur_script_mappings[TASK_CONFIG.JOINT_POSITIONS], joint_positions)
    return signature_str

def __target_pose_to_signature(target_pose: list):
    signature_str = __create_comma_seperated_signature(TASK_CONFIG.ur_script_mappings[TASK_CONFIG.TARGET_POSE], target_pose)
    return signature_str

def __create_comma_seperated_signature (func_name, args, start_bracket="(", end_bracket=")"):
    signature_str = func_name + start_bracket
    for i, arg in enumerate(args):
        signature_str += arg + "," if i != len(args) - 1 else arg
    signature_str += end_bracket
    return signature_str

def __str_to_arr(input: str):
    return input.split(";")

out = parse_task()
print(out)


# script_move = b"""
# def move(): \n
# \ti=0
# \twhile i < 5:  
# \t\tmovej([3.14, -1.5196582, 0.0001, -1.53589, 0.006, 0.02391101], a=0.3 , v=0.6) \n
# \t\tmovej([5, -1.5196582, -0.0959931, -1.53589, 0.006, 0.02391101], a=0.3 , v=0.6) \n
# \t\tmovej([0.004886922, -1.5196582, -0.0959931, -1.53589, 3.14, 0.02391101], a=0.3 , v=0.6) \n
# \t\ti=i+1 \n
# \tend
# end \n
# move() \n
# """
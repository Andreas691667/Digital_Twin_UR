#!/usr/bin/env python

import sys
sys.path.append('..')
import logging
import time

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

#BUILD XML CONFIG BASED ON COMMAND LINE ARGS
arg = str(sys.argv[1])

xml1 = """<?xml version="1.0"?>
<rtde_config>
	<recipe key="outputs">
		<field name="input_bit_register_"""

xml2 = """" type="BOOL"/>
	</recipe>
	<recipe key="inputs">
		<field name="input_bit_register_""" 

xml3 = """" type="BOOL"/>
	</recipe>
</rtde_config>
"""

xml = xml1+arg+xml2+arg+xml3
config_filename = 'example_boolreg_configuration.xml'
file1 = open(config_filename, "w") 
file1.write(xml)
file1.close()



logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '192.168.0.11'
ROBOT_PORT = 30004

keep_running = True
logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
output_names, output_types = conf.get_recipe('outputs')
input_names, input_types = conf.get_recipe('inputs')


con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()


# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(output_names, output_types)
inputs = con.send_input_setup(input_names, input_types)


#start data synchronization
if not con.send_start():
    sys.exit()

#inputs.standard_digital_output_mask = 1

print("running now")

# control loop
while keep_running:
    # receive the current state
    outputs = con.receive()
    
    if outputs is None:
        break

    reg = 'input_bit_register_'+arg
    reg_value = getattr(outputs,reg)
    setattr(inputs,reg,not reg_value)
    print(reg+": "+str(reg_value))

    con.send(inputs)
    break

con.send_pause()
con.disconnect()
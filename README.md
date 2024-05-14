# Digital_Twin_UR
This repository contains the source code for the Bachelor's project made by:

- Andreas Kaag Thomsen
- Buster Salomon Rasmussen

For the Bachelor of Science in the Computer Engineering program at the University of Aarhus. The repository contains the source code for the Digital Twin of the Universal Robots UR3e robot arm as well as the source code for controlling and monitoring the arm.

For handling the connections to the robot, the repository ``URInterface`` has been used, and can be found [here](https://gitlab.au.dk/clagms/urinterface/-/tree/data_publisher/src?ref_type=heads) has been used. Specifically, the ``data_publisher`` branch, which also features a visulisation application. The repository uses the [RTDE Client library](https://github.com/UniversalRobots/RTDE_Python_Client_Library) provided by Universal Robots A/S. 

## How to run the system
To run the system, open two terminals in the root of the repo and run the following commands in each terminal:
**Terminal 1 (Digital Twin)**
```bash
cd src/dt
python digital_ur_main.py -a <approach> -ms <mitigation_strategy> -key <file_key>
```
The following arguments are required:
- -a: The approach to use for detecting missing blocks. Options are: '1' for the first approach using timing thresholds and '2' for the second approach based on model divergence. When using approach 2, the visualisation application will spawn automatically. **The port number 5556 must be manually inputted here**.
- -ms: The mitigation strategy to use for handling missing blocks. Options are: 'shift' for shifting the block origins and 'stock' for using the stock.
- -key (optional): The key to prepend on log files.

**Terminal 2 (Robot Connection Manager)**
```bash
cd src/robot_connection_manager
python rcm_main.py -t <task_name> -key <file_key>
```
The following arguments are required:
- t: The name of the task to execute. Pre-configured tasks are in the folder `src/config/tasks`. The type postfix ``.yaml`` must be omitted.
- -key (optional): The key to prepend on log files.

The system will ask for the operator to press '2' to start the task execution when ready. 

### Important note
The robot must be in the home position before starting the system. This can be done by running the command:
```bash
python rcm_main.py -home True
```
Also, ensure that the robot is set to Remote Control on the Teach Pendant. Please see this [link](https://robodk.com/doc/en/Robots-Universal-Robots-How-enable-Remote-Control-URe.html) on how to do this.

## How to plot trajectories
If approach 2 is used, the DT's planned trajectory is saved to a file and can be plotted along with the PT's trajectory and the real-time-measured error. To plot the trajectories, run the following command in the root of the repo:
```bash
cd data_analysis
python trajectory_plotter.py -key <file_key>
```
The following argument is required:
- -key: The log file key used when running the system.

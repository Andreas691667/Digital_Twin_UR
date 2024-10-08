#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import matplotlib.pyplot as plt
import argparse
import numpy as np
import pandas as pd

def convert_ndarray_from_radians_to_degrees (array_in: np.ndarray) -> np.ndarray:
    return array_in
    # return np.array([(180 / np.pi)*x for x in array_in])

def read_qs(r:pd.DataFrame, start_indx = 0):
    q0_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_0.astype(float).to_list()[start_indx:])
    q1_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_1.astype(float).to_list()[start_indx:])
    q2_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_2.astype(float).to_list()[start_indx:])
    q3_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_3.astype(float).to_list()[start_indx:])
    q4_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_4.astype(float).to_list()[start_indx:])
    q5_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_5.astype(float).to_list()[start_indx:])
    return q0_floats, q1_floats, q2_floats, q3_floats, q4_floats, q5_floats

def read_qds(r:pd.DataFrame):
    qd0_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_0.astype(float))
    qd1_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_1.astype(float))
    qd2_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_2.astype(float))
    qd3_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_3.astype(float))
    qd4_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_4.astype(float))
    qd5_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_5.astype(float))
    return qd0_floats, qd1_floats, qd2_floats, qd3_floats, qd4_floats, qd5_floats

def read_ts(r:pd.DataFrame, start_time=0):
    timestamp_floats = r.timestamp.astype(float)
    # timestamp_float type is pandas.core.series.Series
    # convert to list
    timestamp_floats = timestamp_floats.tolist()
    start_indx = 0
    if start_time != 0:
        start_indx = timestamp_floats.index(start_time)
        timestamp_floats = timestamp_floats[start_indx:]

    timestamp_floats = [x - timestamp_floats[0] for x in timestamp_floats]
    return timestamp_floats, start_indx

if __name__ == "__main__":
   
    parser = argparse.ArgumentParser(description='Trajectory Plotter')
    parser.add_argument('-key', type=str, help='The key prepended to log files')
    args = parser.parse_args()
    file_key = args.key

    file_name_key = file_key if file_key is not None else "E8"
    epsilon = 0.7
    epsilon_t = 1

    # ----- WITH KEY -----
    r_dt = pd.read_csv(f"../src/dt/dt_trajectories/{file_name_key}_dt_trajectory.csv", delimiter=' ')
    error = pd.read_csv(f"../src/dt/error_logs/{file_name_key}_dt_error_log.csv", delimiter=' ')
    # robot data file
    r_pt = pd.read_csv(f"../src/robot_connection_manager/robot_output/{file_name_key}_robot_output.csv", delimiter=' ')

    # simulation data file
    # r_pt = pd.read_csv("../src/robot_connection_manager/rcm_simulator/simulation_data/square.csv", delimiter=' ')
    

    # ----- WITHOUT KEY AND NAME (MANUAL) -----
    # r_dt = pd.read_csv("test_results/dt_trajectories/2_blocks_trajectory_sim_200hz.csv", delimiter=' ')
    # r_pt = pd.read_csv("test_results/robot_data/robot_output_2_blocks_sim.csv", delimiter=' ')
    # error = pd.read_csv("test_results/error_logs/error_log_sim_200hz.csv", delimiter=' ')

    error_ts = error.iloc[:, 0]
    start_time = error_ts[0]
    error_ts = [x - error_ts[0] for x in error_ts]
    error_q0 = error.iloc[:, 1]
    error_q1 = error.iloc[:, 2]
    error_q2 = error.iloc[:, 3]
    error_q3 = error.iloc[:, 4]
    error_q4 = error.iloc[:, 5]
    error_q5 = error.iloc[:, 6]
    fault_q0 = error.iloc[:, 7]
    fault_q1 = error.iloc[:, 8]
    fault_q2 = error.iloc[:, 9]
    fault_q3 = error.iloc[:, 10]
    fault_q4 = error.iloc[:, 11]
    fault_q5 = error.iloc[:, 12]

    erros = [error_q0, error_q1, error_q2, error_q3, error_q4, error_q5]
    faults = [fault_q0, fault_q1, fault_q2, fault_q3, fault_q4, fault_q5]
    # get max error
    max_error = max([max(x) for x in erros])

    timestamp_floats_dt,_ = read_ts(r_dt)
    q0_floats_dt, q1_floats_dt, q2_floats_dt, q3_floats_dt, q4_floats_dt, q5_floats_dt = read_qs(r_dt)
    dt_qs = [q0_floats_dt, q1_floats_dt, q2_floats_dt, q3_floats_dt, q4_floats_dt, q5_floats_dt]
    # qd0_floats_dt, qd1_floats_dt, qd2_floats_dt, qd3_floats_dt, qd4_floats_dt, qd5_floats_dt = read_qds(r_dt)

    timestamp_floats_pt, start_idx = read_ts(r_pt, start_time)
    q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt = read_qs(r_pt, start_idx)
    pt_qs = [q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt]
    qd0_floats_pt, qd1_floats_pt, qd2_floats_pt, qd3_floats_pt, qd4_floats_pt, qd5_floats_pt = read_qds(r_pt)
    
    object_grip = r_pt.output_bit_register_66.astype(float).to_list()

    # make 1 one large figure with 6 subplots with 3 axes each (q_dt, q_pt, q_error)
    # Create a large figure
    large_fig = plt.figure(figsize=(11, 7))

    # Create 6 subfigs
    subfigs = large_fig.subfigures(2,1)
    subfigs = subfigs.ravel()

    # # # set font to serif
    # plt.rcParams['font.family'] = 'Serif'
    # from matplotlib import rc
    # # rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
    # rc('font',**{'family':'serif','serif':['Computer Modern']})
    # rc('text', usetex=True)

    plt.rcParams.update({
    "text.usetex": True,
    "font.family": 'CMU Serif',
    "font.size": 22,
    "figure.dpi": 400,
    "savefig.dpi": 400,
    })

    current_block = 0
    last_object_detected = False
    last_block_at_fault = -1
    first_fault_time = -1
    fault_duration = 0


    for i in range(6):
        if i == 0 or i == 5:
            j = 0 if i == 0 else 1 if i == 2 else 1
            # i = 0 if i == 0 else 2 if i == 2 else 5
            # j=i
            joint_titles = ["Base Joint", "joint 1", "joint 2", "joint 3", "joint 4", "joint 5"]
            subfigs[j].suptitle(f"Position and Error in {joint_titles[i]}")
            axs = subfigs[j].subplots(2, 1, sharex=True)
            axs = axs.ravel()
            axs[0].set_ylabel("[rad]")
            axs[1].set_ylabel("[rad]")
            axs[1].set_xlabel("Time [s]")

            # get max q value
            max_q = max(max(dt_qs[i]), max(pt_qs[i]))
            # get min q value
            min_q = min(min(dt_qs[i]), min(pt_qs[i]))
            axs[0].plot(timestamp_floats_dt, dt_qs[i], color='blue', marker='.', linestyle='', markersize=2)
            axs[0].plot([], [], color='blue', label='DT', marker='.', linestyle='dotted', markersize=2)

            axs[0].plot(timestamp_floats_pt, pt_qs[i], label="PT", color='green', linewidth=4, alpha=0.4)

            axs[1].scatter(error_ts, erros[i], color='black', s=2)
            axs[1].plot([], [], color='black', label='Error', marker='.', linestyle='dotted', markersize=2)
            # plot red dotted horizontal line at epsilon
            axs[1].axhline(y=epsilon, color='red', linestyle='--', label=f"$\epsilon_e$: {epsilon} rad")

            # set axs[2] y axis limits
            axs[0].set_ylim([min_q-0.1, max_q+0.1])
            # axs[1].set_ylim([min_q-0.1, max_q+0.1])
            axs[1].set_ylim([-0.01, max_error+0.1])

            # iterate over faults and plot vertical lines every time a fault occurs
            # if the fault persists for more than epsilon_t seconds, plot a red vertical line
            for j in range(len(faults[i])):
                # check object grip
                object_detected = object_grip[j]
                object_grapped = not last_object_detected and object_detected
                last_object_detected = object_detected

                if object_grapped:
                    current_block += 1

                if faults[i][j]:
                    # if i == 0:
                    #     print(f"Fault detected at {error_ts[j]}s in block {current_block}")
                    #     if file_key == "E6":
                    #         if error_ts[j] == 46.19999999999891 or error_ts[j] == 74.79999999999927:
                    #             last_block_at_fault = current_block
                    #     elif file_key == "E8":
                    #         if error_ts[j] == 45.30000000000018:
                    #             last_block_at_fault = current_block
                    #         elif error_ts[j] > 55:
                    #             last_block_at_fault = -1

                    if first_fault_time == -1:
                        first_fault_time = error_ts[j]
                        fault_duration = 0


                    if current_block != last_block_at_fault:
                        # if fault has persisted for more than epsilon_t seconds
                        if abs(error_ts[j] - first_fault_time) >= epsilon_t:
                            # add vertical line at fault time with opacity
                            axs[0].axvline(x=error_ts[j], color='red',  linewidth=2)
                            axs[1].axvline(x=error_ts[j], color='red',  linewidth=2)
                            first_fault_time = -1
                            last_block_at_fault = current_block

                        # else:
                        #     # add vertical line at fault time with opacity
                        #     axs[0].axvline(x=error_ts[j], color='green', alpha=0.2, linewidth=0.5)
                        #     axs[1].axvline(x=error_ts[j], color='green', alpha=0.2, linewidth=0.5)
                        

                    # same block is at fault again
                    else:
                        if abs(error_ts[j] - first_fault_time) >= epsilon_t:
                            # add vertical line at fault time with opacity
                            axs[0].axvline(x=error_ts[j], color='red',  linewidth=2)
                            axs[1].axvline(x=error_ts[j], color='red',  linewidth=2)
                            first_fault_time = -1
                            last_block_at_fault = -1
                            
                        # else:
                        #     axs[0].axvline(x=error_ts[j], color='blue', alpha=0.2, linewidth=0.5)
                        #     axs[1].axvline(x=error_ts[j], color='blue', alpha=0.2, linewidth=0.5)

                    fault_duration = abs(error_ts[j] - first_fault_time)

                else:
                    first_fault_time = -1
                    block_at_fault = -1


            # add custom legend to green vertical lines and red vertical lines
            # axs[0].plot([], [], color='green', label='Error', alpha=0.5)
            axs[0].plot([], [], color='red', label='Fault detected')
            # axs[1].plot([], [], color='green', label='Error')
            axs[1].plot([], [], color='red', label='Fault detected')
            # axs[0].plot([], [], color='cyan', label='Stock pos. empty')
            # axs[1].plot([], [], color='cyan', label='Stock pos. empty')

            # for j in range(len(faults[i])):
            #     if faults[i][j]:
            #         # add vertical line at fault time with opacity
            #         axs[0].axvline(x=error_ts[j], color='green', alpha=0.2, linewidth=0.5)
            #         axs[1].axvline(x=error_ts[j], color='green', alpha=0.2, linewidth=0.5)

            # add custom legend to green vertical lines
            # axs[0].plot([], [], color='green', label='Fault', alpha=0.5)
            # axs[1].plot([], [], color='green', label='Fault', alpha=0.5)


            # add legend to all subplots and align to the right
            for ax in axs:
                # set axes granularity
                max_time = max(timestamp_floats_dt[-1], timestamp_floats_pt[-1])
                ax.set_xticks(np.arange(0, max_time, 10))
                # ax.set_xticks(np.arange(0, max_time, 0.05))
                ax.legend(loc='upper right')
                ax.grid()






    # fig, axs = plt.subplots(3, 2, sharex=True, sharey=True)
    # fig.suptitle("Joint Positions, Velocities and Errors")
    # fig.subplots_adjust(hspace=0.5)
    # fig.subplots_adjust(wspace=0.5)
    # fig.set_size_inches(10, 10)

    # # plot positions
    # for i in range(6):
    #     # axs[i, :].grid()
    #     # axs[i, :].set_xlabel("Time [s]")
    #     axs[i, 0].set_ylabel("Joint Position [deg]")
    #     axs[i, 1].set_ylabel("Joint Position [deg]")
    #     # axs[i, j].set_title(f"q{i}")
    #     axs[i, 0].plot(timestamp_floats_dt, dt_qs[i], label="DT")
    #     axs[i, 1].plot(timestamp_floats_pt, pt_qs[i], label="PT")
    #     axs[i, 2].plot(error_ts, error_q0, label="Error")
    #     # axs[i, :].legend()    
    
    # # # plot positions
    # fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, sharey=True)
    # ax1.plot(timestamp_floats_dt, q0_floats_dt, label="q0")
    # ax1.plot(timestamp_floats_dt, q1_floats_dt, label="q1")
    # ax1.plot(timestamp_floats_dt, q2_floats_dt, label="q2")
    # ax1.plot(timestamp_floats_dt, q3_floats_dt, label="q3")
    # ax1.plot(timestamp_floats_dt, q4_floats_dt, label="q4")
    # ax1.plot(timestamp_floats_dt, q5_floats_dt, label="q5")
    # # cursor = Cursor(ax1, useblit=True, color='red', linewidth=2)
    
    # ax1.set_xlabel("Time [s]")
    # ax1.set_ylabel("Joint Position [rad]")
    # ax1.set_title("DT")
    
    # ax2.plot(timestamp_floats_pt, q0_floats_pt, label="q0")
    # ax2.plot(timestamp_floats_pt, q1_floats_pt, label="q1")
    # ax2.plot(timestamp_floats_pt, q2_floats_pt, label="q2")
    # ax2.plot(timestamp_floats_pt, q3_floats_pt, label="q3")
    # ax2.plot(timestamp_floats_pt, q4_floats_pt, label="q4")
    # ax2.plot(timestamp_floats_pt, q5_floats_pt, label="q5")
    # cursor = MultiCursor(None, (ax1,ax2), useblit=True, color='red', linewidth=1)

    # ax2.set_xlabel("Time [s]")
    # ax2.set_ylabel("Joint Position [rad]")
    # ax2.set_title("PT")

    # # increase granularity of x-axis
    # ax2.set_xticks(np.arange(0, 25, 1))
    # ax1.grid()
    # ax2.grid()
    # fig.legend()

    # # plot errors
    # figt, axt = plt.subplots(6, 1, sharex=True, sharey=True)
    # axt[0].plot(error_ts, error_q0, label="q0")
    # axt[1].plot(error_ts, error_q1, label="q1")
    # axt[2].plot(error_ts, error_q2, label="q2")
    # axt[3].plot(error_ts, error_q3, label="q3")
    # axt[4].plot(error_ts, error_q4, label="q4")
    # axt[5].plot(error_ts, error_q5, label="q5")

    # axt[5].set_xlabel("Time [s]")
    # axt[0].set_ylabel("Joint Position Error [rad]")
    # axt[0].set_title("Errors")

    # axt[0].set_xticks(np.arange(0, 25, 1))

    # figt.legend()

    # plt.show()

    # # # plot velocities
    # fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, sharey=True)
    # ax1.plot(timestamp_floats_dt, qd0_floats_dt, label="q0")
    # ax1.plot(timestamp_floats_dt, qd1_floats_dt, label="q1")
    # ax1.plot(timestamp_floats_dt, qd2_floats_dt, label="q2")
    # ax1.plot(timestamp_floats_dt, qd3_floats_dt, label="q3")
    # ax1.plot(timestamp_floats_dt, qd4_floats_dt, label="q4")
    # ax1.plot(timestamp_floats_dt, qd5_floats_dt, label="q5")
    # cursor = Cursor(ax1, useblit=True, color='red', linewidth=2)
    
    # ax1.set_xlabel("Time [s]")
    # ax1.set_ylabel("Joint Velocity [rad/s]")
    # ax1.set_title("DT")
    
    # ax2.plot(timestamp_floats_pt, qd0_floats_pt, label="q0")
    # ax2.plot(timestamp_floats_pt, qd1_floats_pt, label="q1")
    # ax2.plot(timestamp_floats_pt, qd2_floats_pt, label="q2")
    # ax2.plot(timestamp_floats_pt, qd3_floats_pt, label="q3")
    # ax2.plot(timestamp_floats_pt, qd4_floats_pt, label="q4")
    # ax2.plot(timestamp_floats_pt, qd5_floats_pt, label="q5")
    # cursor = Cursor(ax2, useblit=True, color='red', linewidth=2)

    # ax2.set_xlabel("Time [s]")
    # ax2.set_ylabel("Joint Velocoty [rad/s]")
    # ax2.set_title("PT")

    # # increase granularity of x-axis
    # ax2.set_xticks(np.arange(0, 25, 1))

    # fig.legend()
    # resolution_value = 1200
    # plt.savefig("myImage.png", format="png", dpi=resolution_value)
    # large_fig.tight_layout()
    # large_fig.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9)
    plt.show()

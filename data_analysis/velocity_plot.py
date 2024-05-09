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

    print(timestamp_floats[0])
    timestamp_floats = [x - timestamp_floats[0] for x in timestamp_floats]
    return timestamp_floats, start_indx

if __name__ == "__main__":
   
    # ----- WITHOUT KEY AND NAME (MANUAL) -----
    r_dt = pd.read_csv("../src/dt/dt_trajectories/VEL_dt_trajectory.csv", delimiter=' ')
    r_pt = pd.read_csv("../src/robot_connection_manager/rcm_simulator/simulation_data/square.csv", delimiter=' ')

    timestamp_floats_dt,_ = read_ts(r_dt)
    q0_floats_dt, q1_floats_dt, q2_floats_dt, q3_floats_dt, q4_floats_dt, q5_floats_dt = read_qs(r_dt)
    dt_qs = [q0_floats_dt, q1_floats_dt, q2_floats_dt, q3_floats_dt, q4_floats_dt, q5_floats_dt]
    qd0_floats_dt, qd1_floats_dt, qd2_floats_dt, qd3_floats_dt, qd4_floats_dt, qd5_floats_dt = read_qds(r_dt)
    dt_qds = [qd0_floats_dt, qd1_floats_dt, qd2_floats_dt, qd3_floats_dt, qd4_floats_dt, qd5_floats_dt]

    timestamp_floats_pt, start_idx = read_ts(r_pt)
    q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt = read_qs(r_pt, start_idx)
    pt_qs = [q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt]
    qd0_floats_pt, qd1_floats_pt, qd2_floats_pt, qd3_floats_pt, qd4_floats_pt, qd5_floats_pt = read_qds(r_pt)
    pt_qds = [qd0_floats_pt, qd1_floats_pt, qd2_floats_pt, qd3_floats_pt, qd4_floats_pt, qd5_floats_pt]
    
    # make 1 one large figure with 6 subplots with 3 axes each (q_dt, q_pt, q_error)
    # Create a large figure
    large_fig = plt.figure()

    # Create 6 subfigs
    subfigs = large_fig.subfigures(1, 1)
    # subfigs = subfigs.ravel()

    # set font to serif
    plt.rcParams['font.family'] = 'serif'

    for i in range(6):
        if i == 0:
            j = 0 if i == 0 else 1 if i == 2 else 2
            i = 0 if i == 0 else 2 if i == 2 else 5

            subfigs.suptitle(f"Velocity in joint {i}", fontsize=20)
            axs = subfigs.subplots(1, 1, sharex=True)
            axs.set_ylabel("[rad/s]", fontsize=18)
            axs.set_xlabel("Time [s]",  fontsize=18)

            # get max q value
            max_q = max(max(dt_qs[i]), max(pt_qs[i]))
            # get min q value
            min_q = min(min(dt_qs[i]), min(pt_qs[i]))
            axs.scatter(timestamp_floats_dt, dt_qds[i], label="DT", color='blue', s=3)


            axs.plot(timestamp_floats_pt, pt_qds[i], label="PT", color='red', linewidth=3, alpha=0.4)

            # diff_err = abs(dt_qs[i] - pt_qs[i]


            # set axes granularity
            max_time = max(timestamp_floats_dt[-1], timestamp_floats_pt[-1])
            axs.set_xticks(np.arange(0, max_time, 10))
            axs.legend(loc='upper right', fontsize=16)
            axs.grid()






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
    plt.show()

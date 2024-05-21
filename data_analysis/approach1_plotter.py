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

def read_ts(r:pd.DataFrame, start_time=0, errors=[]):
    timestamp_floats = r.timestamp.astype(float)
    # timestamp_float type is pandas.core.series.Series
    # convert to list
    timestamp_floats = timestamp_floats.tolist()
    start_indx = 0
    if start_time != 0:
        start_indx = timestamp_floats.index(start_time)
        timestamp_floats = timestamp_floats[start_indx:]

    errors = [x - timestamp_floats[0] for x in errors]
    timestamp_floats = [x - timestamp_floats[0] for x in timestamp_floats]
    return timestamp_floats, start_indx, errors

if __name__ == "__main__":
   
    parser = argparse.ArgumentParser(description='Trajectory Plotter')
    parser.add_argument('-key', type=str, help='The key prepended to log files')
    args = parser.parse_args()
    file_key = args.key

    file_name_key = file_key if file_key is not None else "E3"
    epsilon = 0.7
    errors = [1847.156, 1863.136, 1870.216]

    # ----- WITH KEY -----
    # r_dt = pd.read_csv(f"../src/dt/dt_trajectories/{file_name_key}_dt_trajectory.csv", delimiter=' ')
    # error = pd.read_csv(f"../src/dt/error_logs/{file_name_key}_dt_error_log.csv", delimiter=' ')
    # robot data file
    r_pt = pd.read_csv(f"../src/robot_connection_manager/robot_output/{file_name_key}_robot_output.csv", delimiter=' ')

    timestamp_floats_pt, start_idx, errors = read_ts(r_pt, errors=errors)
    q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt = read_qs(r_pt, start_idx)
    pt_qs = [q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt]
    qd0_floats_pt, qd1_floats_pt, qd2_floats_pt, qd3_floats_pt, qd4_floats_pt, qd5_floats_pt = read_qds(r_pt)
    


    # make 1 one large figure with 6 subplots with 3 axes each (q_dt, q_pt, q_error)
    # Create a large figure
    large_fig = plt.figure()

    # Create 6 subfigs
    subfigs = large_fig.subfigures(2, 1)
    subfigs = subfigs.ravel()

    # set font to serif
    plt.rcParams['font.family'] = 'serif'

    for i in range(6):
        if i == 0 or i == 5:
            j = 0 if i == 0 else 1 if i == 2 else 1
            i = 0 if i == 0 else 2 if i == 2 else 5

            subfigs[j].suptitle(f"Position and faults in joint {i}")
            axs = subfigs[j].subplots(1, 1, sharex=True)
            axs.set_ylabel("[rad]")
            axs.set_ylabel("[rad]")
            axs.set_xlabel("Time [s]")


            axs.plot(timestamp_floats_pt, pt_qs[i], label="PT", color='green', linewidth=3, alpha=0.4)

            # vertical green lines at errors
            for error in errors:
                axs.axvline(x=error, color='red', linewidth=2)

            # add custom legend to green
            axs.plot([], [], label="Fault detected", color='red', linewidth=2)
            axs.legend(loc='upper right')            

            # set x-axis limits
            max_time = timestamp_floats_pt[-1]
            axs.set_xticks(np.arange(0, max_time, 10))

            # add grid
            axs.grid(True)

    plt.show()

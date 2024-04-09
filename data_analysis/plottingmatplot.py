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
from matplotlib.widgets import Cursor

import numpy as np
import pandas as pd

def convert_ndarray_from_radians_to_degrees (array_in: np.ndarray) -> np.ndarray:
    return array_in
    # return np.array([(180 / np.pi)*x for x in array_in])

def read_qs(r:pd.DataFrame):
    q0_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_0.astype(float))
    q1_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_1.astype(float))
    q2_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_2.astype(float))
    q3_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_3.astype(float))
    q4_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_4.astype(float))
    q5_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_5.astype(float))
    return q0_floats, q1_floats, q2_floats, q3_floats, q4_floats, q5_floats

def read_qds(r:pd.DataFrame):
    qd0_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_0.astype(float))
    qd1_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_1.astype(float))
    qd2_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_2.astype(float))
    qd3_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_3.astype(float))
    qd4_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_4.astype(float))
    qd5_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_5.astype(float))
    return qd0_floats, qd1_floats, qd2_floats, qd3_floats, qd4_floats, qd5_floats

def read_ts(r:pd.DataFrame):
    timestamp_floats = r.timestamp.astype(float)
    timestamp_floats = [x - timestamp_floats[0] for x in timestamp_floats]
    return timestamp_floats

if __name__ == "__main__":
   
    r_dt = pd.read_csv("test_results/trajectory_dt.csv", delimiter=' ')
    r_pt = pd.read_csv("test_results/robot_output_two_blocks.csv", delimiter=' ')

    timestamp_floats_dt = read_ts(r_dt)
    q0_floats_dt, q1_floats_dt, q2_floats_dt, q3_floats_dt, q4_floats_dt, q5_floats_dt = read_qs(r_dt)
    # qd0_floats_dt, qd1_floats_dt, qd2_floats_dt, qd3_floats_dt, qd4_floats_dt, qd5_floats_dt = read_qds(r_dt)

    timestamp_floats_pt = read_ts(r_pt)
    q0_floats_pt, q1_floats_pt, q2_floats_pt, q3_floats_pt, q4_floats_pt, q5_floats_pt = read_qs(r_pt)
    
    # timestamp_floats = r.timestamp.astype(float)
    # timestamp_floats = [x - timestamp_floats[0] for x in timestamp_floats]
    # q0_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_0.astype(float))
    # q1_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_1.astype(float))
    # q2_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_2.astype(float))
    # q3_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_3.astype(float))
    # q4_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_4.astype(float))
    # q5_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_5.astype(float))
    # qd0_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_0.astype(float))
    # qd1_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_1.astype(float))
    # qd2_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_2.astype(float))
    # qd3_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_3.astype(float))
    # qd4_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_4.astype(float))
    # qd5_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_5.astype(float))

    # # plot
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    ax1.plot(timestamp_floats_dt, q0_floats_dt, label="q0")
    ax1.plot(timestamp_floats_dt, q1_floats_dt, label="q1")
    ax1.plot(timestamp_floats_dt, q2_floats_dt, label="q2")
    ax1.plot(timestamp_floats_dt, q3_floats_dt, label="q3")
    ax1.plot(timestamp_floats_dt, q4_floats_dt, label="q4")
    ax1.plot(timestamp_floats_dt, q5_floats_dt, label="q5")
    cursor = Cursor(ax1, useblit=True, color='red', linewidth=2)
    
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Joint Position [rad]")
    ax1.set_title("DT")
    
    ax2.plot(timestamp_floats_pt, q0_floats_pt, label="q0")
    ax2.plot(timestamp_floats_pt, q1_floats_pt, label="q1")
    ax2.plot(timestamp_floats_pt, q2_floats_pt, label="q2")
    ax2.plot(timestamp_floats_pt, q3_floats_pt, label="q3")
    ax2.plot(timestamp_floats_pt, q4_floats_pt, label="q4")
    ax2.plot(timestamp_floats_pt, q5_floats_pt, label="q5")
    cursor = Cursor(ax2, useblit=True, color='red', linewidth=2)


    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Joint Position [rad]")
    ax2.set_title("PT")

    # increase granularity of x-axis
    ax2.set_xticks(np.arange(0, 25, 1))

    fig.legend()
    plt.show()

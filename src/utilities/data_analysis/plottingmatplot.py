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
import numpy as np
import csv

import sys

# sys.path.append("../urinterface/src/third_party")
sys.path.append("src\\urinterface\\src\\third_party")
import rtde.csv_reader as csv_reader

def convert_ndarray_from_radians_to_degrees (array_in: np.ndarray) -> np.ndarray:
    return array_in
    # return np.array([(180 / np.pi)*x for x in array_in])

if __name__ == "__main__":
   
    with open("test_results/robot_output_dynamic_threshold_long.csv") as csvfile:
        r = csv_reader.CSVReader(csvfile)

    timestamp_floats = r.timestamp.astype(float)
    timestamp_floats = [x - timestamp_floats[0] for x in timestamp_floats]
    q0_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_0.astype(float))
    q1_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_1.astype(float))
    q2_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_2.astype(float))
    q3_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_3.astype(float))
    q4_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_4.astype(float))
    q5_floats = convert_ndarray_from_radians_to_degrees(r.actual_q_5.astype(float))
    qd0_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_0.astype(float))
    qd1_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_1.astype(float))
    qd2_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_2.astype(float))
    qd3_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_3.astype(float))
    qd4_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_4.astype(float))
    qd5_floats = convert_ndarray_from_radians_to_degrees(r.actual_qd_5.astype(float))

    # # plot
    plt.plot(timestamp_floats, q0_floats, label="q0")
    plt.plot(timestamp_floats, q1_floats, label="q1")
    plt.plot(timestamp_floats, q2_floats, label="q2")
    plt.plot(timestamp_floats, q3_floats, label="q3")
    plt.plot(timestamp_floats, q4_floats, label="q4")
    plt.plot(timestamp_floats, q5_floats, label="q5")
    plt.legend()
    plt.show()

#!/usr/bin/env python
import matplotlib.pyplot as plt
import argparse
import numpy as np
import pandas as pd

def convert_ndarray_from_radians_to_degrees(array_in: np.ndarray) -> np.ndarray:
    return array_in

def read_qs(r: pd.DataFrame, start_indx=0):
    return [
        convert_ndarray_from_radians_to_degrees(r[f'actual_q_{i}'].astype(float).to_list()[start_indx:])
        for i in range(6)
    ]

def read_qds(r: pd.DataFrame):
    return [
        convert_ndarray_from_radians_to_degrees(r[f'actual_qd_{i}'].astype(float))
        for i in range(6)
    ]

def read_ts(r: pd.DataFrame, start_time=0):
    timestamp_floats = r.timestamp.astype(float).tolist()
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

    r_dt = pd.read_csv(f"../src/dt/dt_trajectories/{file_name_key}_dt_trajectory.csv", delimiter=' ')
    error = pd.read_csv(f"../src/dt/error_logs/{file_name_key}_dt_error_log.csv", delimiter=' ')
    r_pt = pd.read_csv(f"../src/robot_connection_manager/robot_output/{file_name_key}_robot_output.csv", delimiter=' ')

    error_ts = error.iloc[:, 0]
    start_time = error_ts[0]
    error_ts = [x - error_ts[0] for x in error_ts]
    errors = [error.iloc[:, i] for i in range(1, 7)]
    faults = [error.iloc[:, i] for i in range(7, 13)]
    max_error = max([max(x) for x in errors])

    timestamp_floats_dt, _ = read_ts(r_dt)
    dt_qs = read_qs(r_dt)
    timestamp_floats_pt, start_idx = read_ts(r_pt, start_time)
    pt_qs = read_qs(r_pt, start_idx)
    object_grip = r_pt.output_bit_register_66.astype(float).to_list()

    plt.rcParams.update({
        "text.usetex": True,
        "font.family": 'CMU Serif',
        "font.size": 6,
        "figure.dpi": 100,
        "savefig.dpi": 100,
    })

    large_fig = plt.figure(figsize=(11, 7))
    subfigs = large_fig.subfigures(2, 1).ravel()

    joint_titles = ["Base Joint", "joint 1", "joint 2", "joint 3", "joint 4", "joint 5"]

    for i in range(2):  # Only plot the upper two subplots
        subfigs[i].suptitle(f"Position and Error in {joint_titles[i]}")
        axs = subfigs[i].subplots(2, 1, sharex=True).ravel()
        axs[0].set_ylabel("[rad]")
        axs[1].set_ylabel("[rad]")
        axs[1].set_xlabel("Time [s]")

        max_q = max(max(dt_qs[i]), max(pt_qs[i]))
        min_q = min(min(dt_qs[i]), min(pt_qs[i]))
        axs[0].plot(timestamp_floats_dt, dt_qs[i], color='blue', marker='.', linestyle='', markersize=1, label='DT')
        axs[0].plot(timestamp_floats_pt, pt_qs[i], label="PT", color='green', linewidth=2, alpha=0.4)

        axs[1].scatter(error_ts, errors[i], color='black', s=1, label='Error')
        axs[1].axhline(y=epsilon, color='red', linestyle='--', label=f"$\\epsilon_e$: {epsilon} rad")

        axs[0].set_ylim([min_q - 0.1, max_q + 0.1])
        axs[1].set_ylim([-0.01, max_error + 0.1])

        for j in range(len(faults[i])):
            if faults[i][j]:
                axs[0].axvline(x=error_ts[j], color='red', linewidth=2)
                axs[1].axvline(x=error_ts[j], color='red', linewidth=2)

        for ax in axs:
            max_time = max(timestamp_floats_dt[-1], timestamp_floats_pt[-1])
            ax.set_xticks(np.arange(0, max_time, 10))
            ax.legend(loc='upper right')
            ax.grid()

    plt.show()

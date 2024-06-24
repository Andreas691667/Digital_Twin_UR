import matplotlib.pyplot as plt
import numpy as np
from numpy import pi


JOINT_SPEED_MAX_DEG = 60  # deg/s
JOINT_ACCELERATION_DEG = 80  # deg/s**2
JOINT_SPEED_RAD = np.deg2rad(JOINT_SPEED_MAX_DEG)  # rad/s
JOINT_ACCELERATION_RAD = np.deg2rad(JOINT_ACCELERATION_DEG)  # rad/s**2
D_a = JOINT_SPEED_RAD**2 / (2 * JOINT_ACCELERATION_RAD)


def QUINTIC_POLYNOMIAL_Q(t, d, T):
    """Quintic polynomial for position profile"""
    return d * ((10 * t**3) / T**3 - (15 * t**4) / T**4 + (6 * t**5) / T**5)

def QUINTIC_POLYNOMIAL_QD(t, d, T):
    """Quintic polynomial for velocity profile"""
    return d * ((30*t**2*(t-T)**2)/T**5)

# Trapezoidal position profile
TRAPEZOIDAL_Q = [
    lambda t: 1 / 2 * JOINT_ACCELERATION_RAD * t**2,
    lambda t: JOINT_SPEED_RAD * t
    - (JOINT_SPEED_RAD**2) / (2 * JOINT_ACCELERATION_RAD),
    lambda t, T: (
        2 * JOINT_ACCELERATION_RAD * JOINT_SPEED_RAD * T
        - 2 * JOINT_SPEED_RAD**2
        - JOINT_ACCELERATION_RAD**2 * (t - T) ** 2
    )
    / (2 * JOINT_ACCELERATION_RAD),
]

TRAPEZOIDAL_QD = [
    lambda t: JOINT_ACCELERATION_RAD * t,
    lambda t: JOINT_SPEED_RAD,
    lambda t, T: JOINT_ACCELERATION_RAD * (T-t),
]

if __name__ == "__main__":

    # three distances from 1 to pi
    distances = np.linspace(1, 3.4, 3)

    N = len(distances)

    fig = plt.figure()
    subfigs = fig.subfigures(N, 1)
    subfigs = subfigs.ravel()

    plt.rcParams["font.family"] = "serif"

    for i, distance in enumerate(distances):
        TI_a = JOINT_SPEED_RAD / JOINT_ACCELERATION_RAD
        TI_c = (distance - 2 * D_a) / JOINT_SPEED_RAD
        T = 2 * TI_a + TI_c
        t = np.linspace(0, T, int(T/0.05))

        # evaluate the quintic polynomial and trapezoidal functions
        q_quintic = np.array([QUINTIC_POLYNOMIAL_Q(t_i, distance, T) for t_i in t])
        q_trapezoidal = np.array(
            [
                (
                    TRAPEZOIDAL_Q[0](t_i)
                    if t_i <= TI_a
                    else (
                        TRAPEZOIDAL_Q[1](t_i)
                        if t_i <= TI_a + TI_c
                        else TRAPEZOIDAL_Q[2](t_i, T)
                    )
                )
                for t_i in t
            ]
        )
        error = q_quintic - q_trapezoidal

        # velocity profiles
        qd_quintic = np.array([QUINTIC_POLYNOMIAL_QD(t_i, distance, T) for t_i in t])
        qd_trapezoidal = np.array(
            [
                (
                    TRAPEZOIDAL_QD[0](t_i)
                    if t_i <= TI_a
                    else (
                        TRAPEZOIDAL_QD[1](t_i)
                        if t_i <= TI_a + TI_c
                        else TRAPEZOIDAL_QD[2](t_i, T)
                    )
                )
                for t_i in t
            ]
        )

        # add title to subfigure 
        subfigs[i].suptitle(
            f"Distance traversed: {round(distance, 2)} rad in {round(T, 2)} s ($\\alpha=${JOINT_ACCELERATION_DEG} deg/$s^2$, $\\omega=${JOINT_SPEED_MAX_DEG} deg/s )"
        )

        axs = subfigs[i].subplots(2, 1, sharex=True)
        axs = axs.ravel()

        quintic_q = axs[0].plot(t, q_quintic, label="Quintic polynomial pos.", color="red")
        trapz_q = axs[0].plot(t, q_trapezoidal, label="Trapezoidal pos.", color="blue")
        axs[0].set_ylabel("Position [rad]")
        
        axs_right = axs[0].twinx()
        quintic_qd = axs_right.plot(t, qd_quintic, label="Quintic polynomial vel.", color="red", linestyle="--", alpha=0.5)
        trapz_qd = axs_right.plot(t, qd_trapezoidal, label="Trapezoidal vel.", color="blue", linestyle="--", alpha=0.5)
        axs_right.set_ylabel("Velocity [rad/s]")
        axs_right.legend(loc="lower left")

        # set y-label coordiante
        axs[0].yaxis.set_label_coords(-0.1, 0.5)

        axs[1].plot(
            t, error, label=f"Error, max: {round(max(error),2)} rad", color="black"
        )
        axs[1].set_ylabel("Error [rad]")
        # set y-label coordiante
        axs[1].yaxis.set_label_coords(-0.1, 0.5)
        axs[1].set_xlabel("Time [s]")

        for ax in axs:
            # add regions showing the trapezoidal regions
            # the labels should be within the regions
            reg_acc = ax.axvspan(0, TI_a, color="blue", alpha=0.1)
            reg_con = ax.axvspan(TI_a, TI_a + TI_c, color="green", alpha=0.1)
            reg_dec = ax.axvspan(TI_a + TI_c, T, color="red", alpha=0.1)

            # add grid
            ax.grid(True)
            # add legends. one for the graphs and one for the regions
            graph_leg = ax.legend(loc="upper left")
            reg_leg = ax.legend(
                [reg_acc, reg_con, reg_dec],
                ["Acceleration", "Constant speed", "Deceleration"],
                loc="upper right",
            )
            # add the legends back to the plot
            ax.add_artist(graph_leg)

    plt.show()

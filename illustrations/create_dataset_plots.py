import os
import numpy as np
import matplotlib.pyplot as plt

from movement_datasets import read_fep_dataset

DTYPE = 'float64'
FEP_REAL_DATASET_PATH = "../movement_datasets/fep_state_to_pid-corrected-torque_55s_dataset.csv"
FEP_SIM_DATASET_PATH = "../movement_datasets/fep_state_to_simulated-torque_55s_dataset.csv"
PLOT_OUTPUT_DIR_PATH = "./"
PLOT_FIGSIZE = (20, 10)
PLOT_DPI = 128


def main():
    """"""
    # Ensure Plot directory exists
    os.makedirs(PLOT_OUTPUT_DIR_PATH, exist_ok=True)

    # Read dataset position, velocity, acceleration and real as well as simulated torque arrays
    print("Reading FEP real and simulated datasets...")
    pos, vel, acc, real_tau = read_fep_dataset(FEP_REAL_DATASET_PATH, DTYPE)
    _, _, _, sim_tau = read_fep_dataset(FEP_SIM_DATASET_PATH, DTYPE)

    # Create plots
    print("Creating FEP dataset plots...")

    # Set timesteps to 55000, as 55000 datapoints
    timesteps = np.arange(0, 55, 1e-3)

    plt.figure(figsize=PLOT_FIGSIZE)
    for i in range(7):
        plt.plot(timesteps, pos[:, i], label=f'j{i + 1}', zorder=2)
    plt.axhline(0, color='lightgrey', zorder=1)
    plt.xlabel('Time [s]', fontsize=20)
    plt.xticks(range(0, 60, 5))
    plt.ylabel('q(t)\u1D62 [rad]', fontsize=20)
    plt.yticks([-2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5])
    plt.ylim(-2.75, 2.75)
    plt.legend(loc=1)
    plt.savefig(fname=PLOT_OUTPUT_DIR_PATH + 'fep_dataset_pos_plot.svg',
                dpi=PLOT_DPI,
                bbox_inches='tight',
                format='svg')

    plt.figure(figsize=PLOT_FIGSIZE)
    for i in range(7):
        plt.plot(timesteps, vel[:, i], label=f'j{i + 1}', zorder=2)
    plt.axhline(0, color='lightgrey', zorder=1)
    plt.xlabel('Time [s]', fontsize=20)
    plt.xticks(range(0, 60, 5))
    plt.ylabel('\u0071\u0307(t)\u1D62 [rad/s]', fontsize=20)
    plt.yticks([-2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5])
    plt.ylim(-2.75, 2.75)
    plt.legend(loc=1)
    plt.savefig(fname=PLOT_OUTPUT_DIR_PATH + 'fep_dataset_vel_plot.svg',
                dpi=PLOT_DPI,
                bbox_inches='tight',
                format='svg')

    plt.figure(figsize=PLOT_FIGSIZE)
    for i in range(7):
        plt.plot(timesteps, acc[:, i], label=f'j{i + 1}', zorder=2)
    plt.axhline(0, color='lightgrey', zorder=1)
    plt.xlabel('Time [s]', fontsize=20)
    plt.xticks(range(0, 60, 5))
    plt.ylabel('\u0071\u0308(t)\u1D62 [rad/s\u00b2]', fontsize=20)
    plt.yticks([-2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5])
    plt.ylim(-2.75, 2.75)
    plt.legend(loc=1)
    plt.savefig(fname=PLOT_OUTPUT_DIR_PATH + 'fep_dataset_acc_plot.svg',
                dpi=PLOT_DPI,
                bbox_inches='tight',
                format='svg')

    plt.figure(figsize=PLOT_FIGSIZE)
    for i in range(7):
        plt.plot(timesteps, real_tau[:, i], label=f'j{i + 1}', zorder=2)
    plt.axhline(0, color='lightgrey', zorder=1)
    plt.xlabel('Time [s]', fontsize=20)
    plt.xticks(range(0, 60, 5))
    plt.ylabel('\u03c4(t)\u1D62 [Nm]', fontsize=20)
    plt.yticks([-70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70])
    plt.ylim(-78, 78)
    plt.legend(loc=1)
    plt.savefig(fname=PLOT_OUTPUT_DIR_PATH + 'fep_dataset_pid-corrected-torque_plot.svg',
                dpi=PLOT_DPI,
                bbox_inches='tight',
                format='svg')

    plt.figure(figsize=PLOT_FIGSIZE)
    for i in range(7):
        plt.plot(timesteps, sim_tau[:, i], label=f'j{i + 1}', zorder=2)
    plt.axhline(0, color='lightgrey', zorder=1)
    plt.xlabel('Time [s]', fontsize=20)
    plt.xticks(range(0, 60, 5))
    plt.ylabel('RNEA \u03c4(t)\u1D62 [Nm]', fontsize=20)
    plt.yticks([-70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70])
    plt.ylim(-78, 78)
    plt.legend(loc=1)
    plt.savefig(fname=PLOT_OUTPUT_DIR_PATH + 'fep_dataset_simulated-torque_plot.svg',
                dpi=PLOT_DPI,
                bbox_inches='tight',
                format='svg')

    print("Creation of FEP dataset plots finished")


if __name__ == '__main__':
    main()

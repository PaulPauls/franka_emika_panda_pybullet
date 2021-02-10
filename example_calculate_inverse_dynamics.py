import numpy as np
import pybullet as p
import pybullet_data

from panda_robot import PandaRobot
from movement_datasets import read_fep_dataset, write_fep_dataset

INCLUDE_GRIPPER = True
DTYPE = 'float64'
SAMPLING_RATE = 1e-3  # 1000Hz sampling rate
FEP_MOVEMENT_DATASET_PATH = "./movement_datasets/fep_state_to_pid-corrected-torque_55s_dataset.csv"
FEP_SIM_OUTPUT_DATASET_PATH = "./movement_datasets/fep_state_to_simulated-torque_55s_dataset.csv"


def main():
    """"""

    # Basic Setup of environment
    physics_client_id = p.connect(p.DIRECT)
    p.setTimeStep(SAMPLING_RATE)
    p.setGravity(0, 0, -9.81)

    # Setup plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    # Setup robot
    panda_robot = PandaRobot(include_gripper=INCLUDE_GRIPPER)

    # Read FEP movement dataset, setting in context the current state of the robot (the position and acceleration of
    # all joints) to the acceleration that is desired in order to follow the intended movement. The actual FEP
    # PID-corrected tau values are discared as they are irrelevant since this is solely a simulation and no comparison.
    pos, vel, desired_acc, _ = read_fep_dataset(FEP_MOVEMENT_DATASET_PATH, DTYPE)

    # Set up variables for simulation loop
    dataset_length = pos.shape[0]
    period = 1 / SAMPLING_RATE
    counter_seconds = -1

    # Set up variable to collect simulated tau values
    simulated_tau = np.ndarray(shape=pos.shape, dtype=DTYPE)

    # start simulation loop
    for i in range(dataset_length):
        # Print status update every second of the simulation
        if i % period == 0:
            counter_seconds += 1
            print("Passed time in simulation: {:>4} sec".format(counter_seconds))

        # Select position, velocity and desired acceleration of current datapoint and convert to list as PyBullet does
        # not yet support numpy arrays as parameters. Save resulting simulated tau value in previously initialized
        # numpy array.
        current_pos = pos[i].tolist()
        current_vel = vel[i].tolist()
        current_desired_acc = desired_acc[i].tolist()
        simulated_tau[i] = panda_robot.calculate_inverse_dynamics(current_pos, current_vel, current_desired_acc)

        # Perform simulation step
        p.stepSimulation()

    # Exit Simulation
    p.disconnect()
    print("Simulation end")

    # Save simulated torques alongside their according position, velocity and desired acceleration as a full FEP
    # simulated dataset
    write_fep_dataset(FEP_SIM_OUTPUT_DATASET_PATH,
                      datasets=[pos, vel, desired_acc, simulated_tau],
                      dtype=DTYPE)


if __name__ == '__main__':
    main()

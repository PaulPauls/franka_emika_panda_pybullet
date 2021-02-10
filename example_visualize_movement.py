import time
import pybullet as p
import pybullet_data

from panda_robot import PandaRobot
from movement_datasets import read_fep_dataset

INCLUDE_GRIPPER = True
DTYPE = 'float64'
SAMPLING_RATE = 1e-3  # 1000Hz sampling rate
FEP_MOVEMENT_DATASET_PATH = "./movement_datasets/fep_state_to_pid-corrected-torque_55s_dataset.csv"


def main():
    """"""

    # Basic Setup of environment
    physics_client_id = p.connect(p.GUI)
    p.setTimeStep(SAMPLING_RATE)
    p.setGravity(0, 0, -9.81)

    # Setup plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    # Setup robot
    panda_robot = PandaRobot(include_gripper=INCLUDE_GRIPPER)

    # Read FEP movement dataset, discarding everything except the joint positions for each sampling point as PyBullet
    # can be set to figure joint torques out by itself and only requires desired joint positions.
    pos, _, _, _ = read_fep_dataset(FEP_MOVEMENT_DATASET_PATH, DTYPE)

    # Set up variables for simulation loop
    dataset_length = pos.shape[0]
    period = 1 / SAMPLING_RATE
    counter_seconds = -1

    # start simulation loop
    for i in range(dataset_length):
        # Print status update every second of the simulation
        if i % period == 0:
            counter_seconds += 1
            print("Passed time in simulation: {:>4} sec".format(counter_seconds))

        # Select data of current position, then convert to list as PyBullet does not yet support numpy arrays as
        # parameters
        current_pos = pos[i].tolist()
        panda_robot.set_target_positions(current_pos)

        # Perform simulation step
        p.stepSimulation()
        time.sleep(SAMPLING_RATE)

    # Exit Simulation
    p.disconnect()
    print("Simulation end")


if __name__ == '__main__':
    main()

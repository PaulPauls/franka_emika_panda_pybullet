import time
import pybullet as p
import pybullet_data

from panda_robot import PandaRobot

INCLUDE_GRIPPER = True
DTYPE = 'float64'
SAMPLING_RATE = 1e-3  # 1000Hz sampling rate
SIM_LENGTH_SEC = 60


def main():
    """
    REMARK:
    Due to the I assume initial reset the simulation starts out with some velocity in the joints. It will therefore
    move in the beginning of the simulation. From there on after the simulation however should be properly gravity
    compensating when you move the joints with the mouse, making them stand still or giving them momentum.
    """

    # Basic Setup of environment
    physics_client_id = p.connect(p.GUI)
    p.setTimeStep(SAMPLING_RATE)
    p.setGravity(0, 0, -9.81)

    # Setup plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    # Setup robot
    panda_robot = PandaRobot(include_gripper=INCLUDE_GRIPPER)

    # Set up variables for simulation loop
    period = 1 / SAMPLING_RATE
    counter_seconds = -1
    sim_datapoints = int(SIM_LENGTH_SEC * period)

    # start simulation loop
    for i in range(sim_datapoints):
        # Print status update every second of the simulation
        if i % period == 0:
            counter_seconds += 1
            print("Passed time in simulation: {:>4} sec".format(counter_seconds))

        # Determine current state (position and velocity) of robot. Set the desired acceleration to 0 in order to only
        # compensate for gravity but leave all other movement as is.
        pos, vel = panda_robot.get_position_and_velocity()
        desired_acc = [0. for _ in pos]

        # Determine proper torque for the desired gravity compensation acceleration and set it in the robot
        torques = panda_robot.calculate_inverse_dynamics(pos=pos, vel=vel, desired_acc=desired_acc)
        panda_robot.set_torques(torques)

        # Perform simulation step
        p.stepSimulation()
        time.sleep(SAMPLING_RATE)

    # Exit Simulation
    p.disconnect()
    print("Simulation end")


if __name__ == '__main__':
    main()

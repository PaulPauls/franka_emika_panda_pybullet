import os
import math
import pybullet as p


class PandaRobot:
    """"""

    def __init__(self, include_gripper):
        """"""
        p.setAdditionalSearchPath(os.path.dirname(__file__) + '/model_description')
        panda_model = "panda_with_gripper.urdf" if include_gripper else "panda.urdf"
        self.robot_id = p.loadURDF(panda_model, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

        # Set maximum joint velocity. Maximum joint velocity taken from:
        # https://s3-eu-central-1.amazonaws.com/franka-de-uploads/uploads/Datasheet-EN.pdf
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=0, maxJointVelocity=150 * (math.pi / 180))
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=1, maxJointVelocity=150 * (math.pi / 180))
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=2, maxJointVelocity=150 * (math.pi / 180))
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=3, maxJointVelocity=150 * (math.pi / 180))
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=4, maxJointVelocity=180 * (math.pi / 180))
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=5, maxJointVelocity=180 * (math.pi / 180))
        p.changeDynamics(bodyUniqueId=self.robot_id, linkIndex=6, maxJointVelocity=180 * (math.pi / 180))

        # Set DOF according to the fact that either gripper is supplied or not and create often used joint list
        self.dof = p.getNumJoints(self.robot_id) - 1
        self.joints = range(self.dof)

        # Reset Robot
        self.reset_state()

    def reset_state(self):
        """"""
        for j in range(self.dof):
            p.resetJointState(self.robot_id, j, targetValue=0)
        p.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                    jointIndices=self.joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for _ in self.joints])

    def get_dof(self):
        """"""
        return self.dof

    def get_joint_info(self, j):
        """"""
        return p.getJointInfo(self.robot_id, j)

    def get_base_position_and_orientation(self):
        """"""
        return p.getBasePositionAndOrientation(self.robot_id)

    def get_position_and_velocity(self):
        """"""
        joint_states = p.getJointStates(self.robot_id, self.joints)
        joint_pos = [state[0] for state in joint_states]
        joint_vel = [state[1] for state in joint_states]
        return joint_pos, joint_vel

    def calculate_inverse_kinematics(self, position, orientation):
        """"""
        return p.calculateInverseKinematics(self.robot_id, self.dof, position, orientation)

    def calculate_inverse_dynamics(self, pos, vel, desired_acc):
        """"""
        assert len(pos) == len(vel) and len(vel) == len(desired_acc)
        vector_length = len(pos)

        # If robot set up with gripper, set those positions, velocities and desired accelerations to 0
        if self.dof == 9 and vector_length != 9:
            pos = pos + [0., 0.]
            vel = vel + [0., 0.]
            desired_acc = desired_acc + [0., 0.]

        simulated_torque = list(p.calculateInverseDynamics(self.robot_id, pos, vel, desired_acc))

        # Remove unnecessary simulated torques for gripper if robot set up with gripper
        if self.dof == 9 and vector_length != 9:
            simulated_torque = simulated_torque[:7]
        return simulated_torque

    def set_target_positions(self, desired_pos):
        """"""
        # If robot set up with gripper, set those positions to 0
        if self.dof == 9:
            desired_pos = desired_pos + [0., 0.]
        p.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=desired_pos)

    def set_torques(self, desired_torque):
        """"""
        p.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                    jointIndices=self.joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=desired_torque)

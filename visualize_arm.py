import os
import time
import click
import pybullet as p
import pybullet_data
import numpy as np

import logging


@click.command()
@click.option(
    "--urdf-path",
    type=click.Path(exists=True, dir_okay=False, readable=True),
    required=True,
    help="path to the robot arm urdf file",
)
@click.option(
    "--gui/--no-gui",
    default=True,
    help="enable or disable pybullet gui (default: gui on)",
)
@click.option(
    "--ee-link",
    type=int,
    default=-1,
    help="end-effector link index (default: -1 = last link)",
)
def interactive_move_to_xyz(urdf_path, gui, ee_link):
    """load a urdf and interactively move end-effector via sliders (X,Y,Z)."""

    # connect pybullet
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # world setup
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # load robot urdf
    robot_id = p.loadURDF(
        os.path.abspath(urdf_path), basePosition=[0, 0, 0], useFixedBase=True
    )
    num_joints = p.getNumJoints(robot_id)
    print(f"loaded urdf: {urdf_path}, joints: {num_joints}")

    # interactive sliders
    slider_x = p.addUserDebugParameter("target_x", -0.5, 0.5, 0.1)
    slider_y = p.addUserDebugParameter("target_y", -0.5, 0.5, 0.0)
    slider_z = p.addUserDebugParameter("target_z", 0.0, 0.5, 0.2)

    print("use sliders to move end-effector, ctrl+c to quit")

    try:
        while True:
            # read slider values
            target_x = p.readUserDebugParameter(slider_x)
            target_y = p.readUserDebugParameter(slider_y)
            target_z = p.readUserDebugParameter(slider_z)
            target_pos = [target_x, target_y, target_z]

            # compute IK
            if ee_link == -1:
                ee_link_idx = num_joints - 1
            else:
                ee_link_idx = ee_link

            joint_positions = p.calculateInverseKinematics(
                robot_id, ee_link_idx, target_pos
            )

            # apply position control
            for j in range(len(joint_positions)):
                p.setJointMotorControl2(
                    robot_id, j, p.POSITION_CONTROL, joint_positions[j]
                )

            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    interactive_move_to_xyz()

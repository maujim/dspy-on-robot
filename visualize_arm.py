import os
import time
import json
import click
import numpy as np
import pybullet as p
import pybullet_data


@click.command()
@click.option(
    "--urdf-path",
    type=click.Path(exists=True, dir_okay=False, readable=True),
    required=True,
    help="path to the robot arm urdf file",
)
@click.option(
    "--ee-link",
    type=int,
    default=-1,
    help="end-effector link index (default: last link)",
)
@click.option(
    "--output-json",
    type=click.Path(writable=True),
    default="arm_traces.json",
    help="path to save dataset JSON",
)
def generate_and_validate_traces(urdf_path, ee_link, output_json):
    """generate a small dataset of target positions → joint angles and validate in pybullet sim."""

    # connect pybullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # load robot
    robot_id = p.loadURDF(
        os.path.abspath(urdf_path), basePosition=[0, 0, 0], useFixedBase=True
    )
    num_joints = p.getNumJoints(robot_id)
    print(f"loaded urdf: {urdf_path}, joints: {num_joints}")

    # define targets
    targets = [[0.1, 0.0, 0.15], [0.05, 0.05, 0.12], [-0.05, 0.08, 0.14]]

    dataset = []

    if ee_link == -1:
        ee_link_idx = num_joints - 1
    else:
        ee_link_idx = ee_link

    # helper: move robot joints to desired positions
    def apply_joint_positions(joint_positions, steps=50):
        for t in range(steps):
            alpha = (t + 1) / steps
            for j in range(len(joint_positions)):
                p.setJointMotorControl2(
                    robot_id, j, p.POSITION_CONTROL, joint_positions[j] * alpha
                )
            p.stepSimulation()
            time.sleep(1 / 240.0)

    # generate joint angles and simulate
    for target in targets:
        joint_positions = p.calculateInverseKinematics(robot_id, ee_link_idx, target)
        joint_positions = joint_positions[:num_joints]  # ensure length matches

        dataset.append({"target": target, "joint_angles": joint_positions})

        print(f"moving to target: {target}")
        apply_joint_positions(joint_positions)
        # optional: pause briefly
        time.sleep(0.5)

    # save dataset
    with open(output_json, "w") as f:
        json.dump(dataset, f, indent=2)

    print(f"dataset saved to {output_json}")
    print("simulation complete. press ctrl+c to exit.")
    try:
        while True:
            p.stepSimulation()
            time.sleep(1 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    generate_and_validate_traces()

import time
import os
import click
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
    "--gui/--no-gui",
    default=True,
    help="enable or disable pybullet gui (default: gui on)",
)
def visualize_arm(urdf_path, gui):
    """simple script to load and visualize a urdf arm in pybullet."""

    # connect to pybullet
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # setup world
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # load arm urdf
    base_pos = [0, 0, 0]
    base_orn = [0, 0, 0, 1]  # quaternion
    robot_id = p.loadURDF(
        fileName=os.path.abspath(urdf_path),
        basePosition=base_pos,
        baseOrientation=base_orn,
        useFixedBase=True,
    )

    num_joints = p.getNumJoints(robot_id)
    print(f"loaded urdf: {urdf_path}")
    print(f"number of joints: {num_joints}")
    for j in range(num_joints):
        info = p.getJointInfo(robot_id, j)
        print(f"joint {j}: {info[1].decode('utf-8')} type={info[2]}")

    # spin loop
    print("press ctrl+c to quit")
    try:
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()

if __name__ == "__main__":
    visualize_arm()

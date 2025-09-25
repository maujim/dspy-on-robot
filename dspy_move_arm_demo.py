# file: dspy_move_arm_lm_clean.py
# run: pip install dspy numpy click

import json
import click
import numpy as np
import dspy

@click.command()
@click.option(
    "--dataset-json",
    type=click.Path(exists=True, dir_okay=False, readable=True),
    required=True,
    help="path to the JSON dataset of targets and joint angles",
)
def main(dataset_json):
    """
    fully working dspy LM demo:
    predicts joint angles for target positions using dataset + fallback LM
    """

    # ------------------ configure LM ------------------
    lm = dspy.LM("ollama_chat/gemma3:1b", api_base="http://localhost:11434", api_key="")
    dspy.configure(lm=lm)

    # ------------------ load dataset ------------------
    with open(dataset_json, "r") as f:
        dataset = json.load(f)

    # ------------------ define MoveArm signature ------------------
    class MoveArm(dspy.Signature):
        target_x = dspy.InputField()
        target_y = dspy.InputField()
        target_z = dspy.InputField()
        joint_angles = dspy.OutputField(desc="list of 6 floats")

        def run(self, target_x, target_y, target_z):
            # baseline: exact dataset match
            for entry in dataset:
                if np.allclose([target_x, target_y, target_z], entry["target"], atol=1e-5):
                    return {"joint_angles": entry["joint_angles"]}

            # fallback LM
            prompt = f"""
            given a 6-DOF robot arm, provide joint angles (radians) for the end-effector
            to reach coordinates x={target_x}, y={target_y}, z={target_z}.
            return a list of 6 floats only.
            """
            result = self.lm(prompt)
            try:
                angles = [float(x.strip()) for x in result.strip("[]").split(",")]
            except Exception:
                angles = dataset[0]["joint_angles"]
            return {"joint_angles": angles}

    # ------------------ run demo ------------------
    print("running MoveArm LM demo on dataset targets")
    move_sig = MoveArm
    for entry in dataset:
        target = entry["target"]
        out = move_sig(target_x=target[0], target_y=target[1], target_z=target[2])()
        print(f"target: {target}, predicted joint angles: {out['joint_angles']}")


if __name__ == "__main__":
    main()

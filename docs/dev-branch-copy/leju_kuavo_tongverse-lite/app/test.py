import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.objects.ground_plane import GroundPlane

from tongverselite.agent import Agent

from biped_ctrl_client import BipedCtrlClient  # noqa # pylint: disable=wrong-import-order # isort: skip


def main():
    # Add ground
    GroundPlane(prim_path="/World/groundPlane", name="ground_plane", visible=True)

    # Add light
    prim_utils.create_prim(
        "/World/Light/DistantLight",
        "DistantLight",
        translation=(0, 0, 0),
        attributes={"intensity": 1000.0, "color": (1.0, 1.0, 1.0)},
    )
    _ = Agent()

    client = BipedCtrlClient(ip="0.0.0.0", port=8800)

    msg = {"joint_state": {"elbow": 1.00}}

    while True:
        cmd = client.get_command(msg)
        print(cmd)
        tv.sim.step()

    # app.shutdown()


if __name__ == "__main__":
    main()

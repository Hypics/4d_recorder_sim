from omni.isaac.kit import SimulationApp

CONFIG = {"headless": False, 'width': 1920, 'height': 1080}  # nopep8
simulation_app = SimulationApp(launch_config=CONFIG)  # nopep8

import numpy as np

from omni import physxvehicle
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage
from pxr import PhysxSchema

ENV_URL = "usd/demo_vehicle.usd"

if __name__ == "__main__":
    # enable ROS2 bridge extension
    extensions.enable_extension("omni.isaac.ros2_bridge")
    if CONFIG['headless'] is True:
        extensions.enable_extension("omni.kit.livestream.native")

    step_time = 1.0 / 60.0
    simulation_context = SimulationContext(
        physics_dt=step_time,
        rendering_dt=step_time,
        stage_units_in_meters=1.0,
        backend="torch",
    )

    print(f"Loading Stage {ENV_URL}")
    stage.add_reference_to_stage(
        usd_path=ENV_URL,
        prim_path="/World",
    )
    stage.set_stage_up_axis(
        axis='y'
    )

    simulation_app.update()

    # need to initialize physics getting any articulation..etc
    simulation_context.initialize_physics()

    simulation_context.play()

    while simulation_app.is_running():
        physxVehicleInterface = physxvehicle.get_physx_vehicle_interface()
        steerAngleInnerWheel = 30 /180.0 * np.pi
        steerAngleOuterWheel = physxVehicleInterface.compute_ackermann_steering_angle(
            steerAngleInnerWheel, 0.65, 0.6
        )

        prim = stage.get_current_stage().GetPrimAtPath("/World/Car_0/FrontLeftWheel")
        wheelControllerAPI_FL = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

        prim = stage.get_current_stage().GetPrimAtPath("/World/Car_0/FrontRightWheel")
        wheelControllerAPI_FR = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

        if steerAngleInnerWheel >= 0:
            wheelControllerAPI_FL.GetSteerAngleAttr().Set(steerAngleInnerWheel)
            wheelControllerAPI_FR.GetSteerAngleAttr().Set(steerAngleOuterWheel)
        else:
            wheelControllerAPI_FL.GetSteerAngleAttr().Set(steerAngleOuterWheel)
            wheelControllerAPI_FR.GetSteerAngleAttr().Set(steerAngleInnerWheel)

        simulation_context.step(render=True)

    simulation_context.stop()
    simulation_app.close()

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to spawn prims into the scene.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/00_sim/spawn_prims.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse


from isaaclab.app import AppLauncher
# from pxr import UsdShade, Sdf, UsdGeom, Gf

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab_assets import UR10_CFG

# Create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

def design_scene():
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn distant light
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1, 0, 10))

    # create a new xform prim for all objects to be spawned under
    prim_utils.create_prim("/World/Objects", "Xform")
    # spawn a red cone
    # cfg_cone = sim_utils.ConeCfg(
    #     radius=0.15,
    #     height=0.5,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
    # )
    # cfg_cone.func("/World/Objects/Cone1", cfg_cone, translation=(-1.0, 1.0, 1.0))
    # cfg_cone.func("/World/Objects/Cone2", cfg_cone, translation=(-1.0, -1.0, 1.0))

    #     # spawn a red cone
    # cfg_cone = sim_utils.ConeCfg(
    #     radius=0.15,
    #     height=0.5,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
    # )
    # cfg_cone.func("/World/Objects/Cone1", cfg_cone, translation=(-1.0, 1.0, 1.0))
    # cfg_cone.func("/World/Objects/Cone2", cfg_cone, translation=(-1.0, -1.0, 1.0))

    # spawn a green cone with colliders and rigid body
# ---------------------------------------------------------------------------
    # cfg_cone_rigid = sim_utils.ConeCfg(
    #     radius=0.15,
    #     height=0.5,
    #     rigid_props=sim_utils.RigidBodyPropertiesCfg(),
    #     mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
    #     collision_props=sim_utils.CollisionPropertiesCfg(),
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
    # )
    # cfg_cone_rigid.func(
    #     "/World/Objects/ConeRigid", cfg_cone_rigid, translation=(-0.2, 0.0, 2.0), orientation=(0.5, 0.0, 0.5, 0.0)
    # )
# ---------------------------------------------------------------------------

    friction_material = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.5,  # Adjust as needed
        dynamic_friction=1.0,  # Adjust as needed
        restitution=0.1  # Adjust as needed
    )

    cfg_zone1 = sim_utils.CuboidCfg(
        size=(1.0, 0.25, 0.001),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)), 
        physics_material=friction_material 
        # rigid_props=sim_utils.RigidBodyPropertiesCfg(),  # Define as a rigid body

        # collision_props=sim_utils.CollisionPropertiesCfg(),  # Enable collision
    )

    cfg_zone1.func("/World/Objects/zone1", cfg_zone1, translation=(-0.28873, 0.32219, 1.0485))

    cfg_zone2 = sim_utils.CuboidCfg(
        size=(0.25, 0.25, 0.001),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0)),
        # rigid_props=sim_utils.RigidBodyPropertiesCfg(),  # Define as a rigid body

        # collision_props=sim_utils.CollisionPropertiesCfg(),  # Enable collision
    )

    cfg_zone2.func("/World/Objects/zone2", cfg_zone2, translation=(0.34278, 0.32219, 1.0485))

    # cfg_ur10_robot = UR10_CFG.replace(prim_path="/World/Objects/UR10Robot")
    # cfg_ur10_robot.func("/World/Objects/UR10Robot", cfg_ur10_robot, translation=(0.0, 0.0, 1.05))

    # cfg_cube_rigid = sim_utils.CuboidCfg(
    #     size=(0.01, 0.01, 0.01),
    #     rigid_props=sim_utils.RigidBodyPropertiesCfg(),
    #     mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
    #     collision_props=sim_utils.CollisionPropertiesCfg(),
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),  # Red color
    # )
    # cfg_cube_rigid.func(
    #     "/World/Objects/CubeRigid", cfg_cube_rigid, translation=(-0.2, 0.32219, 1.6), orientation=(0.5, 0.0, 0.5, 0.0)
    # )

    # cfg_cylinder_rigid = sim_utils.CylinderCfg(
    # radius=0.015,
    # height=0.05,
    # rigid_props=sim_utils.RigidBodyPropertiesCfg(),
    # mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
    # collision_props=sim_utils.CollisionPropertiesCfg(),
    # visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),  # Blue color
    # )
    # cfg_cylinder_rigid.func(
    #     "/World/Objects/CylinderRigid", cfg_cylinder_rigid, translation=(-0.2, 0.32219, 1.6), orientation=(0.5, 0.0, 0.5, 0.0)
    # )

    # # Define the capsule configuration
    # cfg_capsule_rigid = sim_utils.CapsuleCfg(
    #     radius=0.015,  # Radius of the capsule
    #     height=0.025,   # Height of the cylindrical part of the capsule
    #     rigid_props=sim_utils.RigidBodyPropertiesCfg(),  # Rigid body properties
    #     mass_props=sim_utils.MassPropertiesCfg(mass=1.0),  # Mass properties
    #     collision_props=sim_utils.CollisionPropertiesCfg(),  # Collision properties
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0)),  # Yellow color
    #     physics_material=friction_material 
    # )

    # # Add the capsule to the scene
    # cfg_capsule_rigid.func(
    #     "/World/Objects/CapsuleRigid",  # Path in the scene
    #     cfg_capsule_rigid,  # Configuration defined above
    #     translation=(-0.1, 0.33219, 1.7),  # Position in the scene
    #     orientation=(0.0, 0.0, 0.0, 1.0)   # Orientation as a quaternion (w, x, y, z)
    # )

  

    # # spawn a blue cuboid with deformable body
    # cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
    #     size=(0.2, 0.5, 0.2),
    #     deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
    #     physics_material=sim_utils.DeformableBodyMaterialCfg(),
    # )
    # cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.15, 0.0, 2.0))

    # spawn a usd file of a table into the scene
    # cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
    # cfg.func("/World/Objects/Table", cfg, translation=(0.0, 0.0, 1.05))
    cfg_table = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
        # collision_props=sim_utils.CollisionPropertiesCfg(),  # Enable collision
        )

    cfg_table.func("/World/Objects/Table", cfg_table, translation=(0.0, 0.0, 1.05))

    # qr_content = "hi"
    # qr_image_path = "/tmp/qr_code.png"
    # generate_qr_code(qr_content, qr_image_path)
    # add_qr_code_to_table(sim_utils.get_current_stage(), "/World/Objects/Table", qr_image_path)


def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])

    # Design scene by adding assets to it
    design_scene()

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

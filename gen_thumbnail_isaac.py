import math
import os

import click
import numpy as np
from PIL import Image


class IsaacRender:
    def __init__(self):
        self.root_prim_path = "/World"
        self.world = None
        self.simulator = self.load_simulator()

    def load_simulator(self):
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": True, "width": 1920, "height": 1080})

        from isaacsim.core.api import World
        import isaacsim.core.utils.prims as prims_utils
        self.world = World(stage_units_in_meters=1.0)
        world_prim = prims_utils.define_prim(prim_path=self.root_prim_path)
        self.world.stage.SetDefaultPrim(world_prim)
        return simulation_app

    def add_object(self, usd_path):
        import isaacsim.core.utils.prims as prim_utils

        prim_path = f"{self.root_prim_path}/object"
        prim_utils.create_prim(
            prim_path=prim_path,
            usd_path=usd_path,
            position=[0, 0, 0],
            orientation=[1, 0, 0, 0],
        )
        self.world.reset()
        return prim_path

    def delete_object(self, prim_path):
        import isaacsim.core.utils.stage as stage_utils

        stage = stage_utils.get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            stage.RemovePrim(prim_path)

def add_light():
    from pxr import UsdLux
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    dome_light_path = "/World/DomeLight"
    dome_light_prim = stage.GetPrimAtPath(dome_light_path)
    if not dome_light_prim.IsValid():
        dome_light_prim = UsdLux.DomeLight.Define(stage, dome_light_path)
        dome_light_prim.GetIntensityAttr().Set(800)


def calculate_camera_pose(center,
                          dimensions,
                          distance_factor: float = 1.1,
                          elevation_deg: float = 30.0,
                          aspect_ratio: float = 1.0,
                          min_fov_y_deg: float = 20.0) -> tuple:
    center = np.array(center, dtype=float)
    dimensions = np.array(dimensions, dtype=float)
    width, height, depth = dimensions
    diagonal = math.sqrt(width**2 + height**2 + depth**2)

    if center[2] < 0.2:
        center[2] += depth / 2

    look = np.array([center[0],center[1]])
    radius = diagonal * 0.5
    safe_aspect = aspect_ratio if aspect_ratio > 0 else 1.0
    fov_y_rad = math.radians(min_fov_y_deg)
    fov_x_rad = 2.0 * math.atan(math.tan(fov_y_rad / 2.0) * safe_aspect)
    fov_min = min(fov_x_rad, fov_y_rad)
    min_distance = radius / max(math.sin(fov_min / 2.0), 1e-6)
    distance = max(min_distance, diagonal * distance_factor)
    elevation_rad = math.radians(elevation_deg)

    dx = math.cos(elevation_rad)
    dz = math.sin(elevation_rad)

    eye = np.array([center[0] + dx * distance, center[1], center[2] + dz * distance])
    return eye, look


def get_visible_mesh_bounds(prim_path) -> tuple:
    from pxr import Usd, UsdGeom
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim.IsValid():
        raise ValueError(f"Invalid prim path: {prim_path}")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )

    min_corner = None
    max_corner = None
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.GetVisibilityAttr().Get() != UsdGeom.Tokens.inherited:
            continue
        if imageable.GetPurposeAttr().Get() == UsdGeom.Tokens.guide:
            continue
        world_bound = bbox_cache.ComputeWorldBound(prim)
        aligned_box = world_bound.ComputeAlignedBox()
        bbox_min = np.array(aligned_box.GetMin())
        bbox_max = np.array(aligned_box.GetMax())
        if min_corner is None:
            min_corner = bbox_min
            max_corner = bbox_max
        else:
            min_corner = np.minimum(min_corner, bbox_min)
            max_corner = np.maximum(max_corner, bbox_max)

    if min_corner is None:
        raise ValueError(f"No visible meshes found under {prim_path}")

    print(max_corner,min_corner)
    center = (min_corner + max_corner) / 2.0
    dimensions = max_corner - min_corner
    return center, dimensions


@click.command()
@click.option("--usd-path", default="/home/galbot/Projects/long_horizon_vla/storage/sn_assets/long_horizon_local/washingmachine/Meidi/model/usd_000/MeiDiXiYiJi.usd", help="USD file path.")
@click.option("--thumbnail-path", default="/home/galbot/Projects/long_horizon_vla/storage/sn_assets/long_horizon_local/washingmachine/Meidi/model/a.png", help="Output thumbnail path.")
@click.option("--width", default=340, show_default=True, type=int)
@click.option("--height", default=235, show_default=True, type=int)
def render_thumbnail(usd_path, thumbnail_path, width, height):
    if not os.path.exists(usd_path):
        raise FileNotFoundError(f"USD file not found: {usd_path}")

    output_dir = os.path.dirname(thumbnail_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    isaac_render = IsaacRender()
    add_light()

    isaac_render.delete_object("/World/object")
    prim_path = isaac_render.add_object(usd_path=usd_path)

    isaac_render.world.step()
    isaac_render.world.step()

    center, dimensions = get_visible_mesh_bounds(prim_path)
    aspect_ratio = width / height if height > 0 else 1.0
    eye, look = calculate_camera_pose(center, dimensions, elevation_deg=30, aspect_ratio=aspect_ratio)

    from omni.isaac.sensor import Camera
    import omni.isaac.core.utils.viewports as viewports_utils

    camera = Camera(
        prim_path="/World/camera",
        position=[0.0, 0.0, 0.0],
        frequency=20,
        resolution=(width, height),
        orientation=[1.0, 0.0, 0.0, 0.0],
    )

    viewports_utils.set_camera_view(
        eye=eye,
        target=np.array(look),
        camera_prim_path="/World/camera",
    )

    isaac_render.world.reset()
    camera.initialize()

    for _ in range(20):
        isaac_render.world.step(render=True)

    thumbnail_img = Image.fromarray(camera.get_rgba()[:, :, :3])
    thumbnail_img.save(thumbnail_path)
    print(f"Thumbnail saved to {thumbnail_path}")

    isaac_render.simulator.close()


if __name__ == "__main__":
    render_thumbnail()

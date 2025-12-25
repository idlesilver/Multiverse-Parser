import os
import tempfile
from typing import Optional

import mujoco
import numpy
from dm_control import mjcf

from .factory import InertiaSource
from .importer.mjcf_importer import MjcfImporter


def _write_mjcf_to_file(root: mjcf.RootElement, directory: str, file_name: str = "scene.xml") -> str:
    """Serialize a dm_control MJCF root to disk.

    Args:
        root: dm_control.mjcf.RootElement representing the assembled MuJoCo scene.
        directory: Temporary directory where the XML should be written.
        file_name: Target XML file name.

    Returns:
        Absolute path to the written XML file.
    """
    xml_path = os.path.join(directory, file_name)
    xml_string = root.to_xml_string()
    if isinstance(xml_string, bytes):
        xml_string = xml_string.decode("utf-8")
    with open(xml_path, "w", encoding="utf-8") as fp:
        fp.write(xml_string)
    return xml_path


def export_mujoco_scene_to_usd(
        mj_model: mujoco.MjModel,
        save_file_path: str,
        *,
        mjcf_root: Optional[mjcf.RootElement] = None,
        fixed_base: bool = False,
        with_physics: bool = True,
        with_visual: bool = True,
        with_collision: bool = True,
        root_name: str = "world",
        inertia_source: InertiaSource = InertiaSource.FROM_SRC,
        default_rgba: Optional[numpy.ndarray] = None,
) -> str:
    """Export the current MuJoCo scene to USD using Multiverse-Parser.

    Args:
        mj_model: Compiled `mujoco.MjModel` corresponding to the scene.
        save_file_path: Destination USD path.
        mjcf_root: The dm_control MJCF root used to author the scene.
        fixed_base: Whether to treat root body as fixed when building USD.
        with_physics: Include physics properties in the exported USD.
        with_visual: Include visual geometry.
        with_collision: Include collision geometry.
        root_name: Root body name inside USD.
        inertia_source: Source for inertia computation.
        default_rgba: Default RGBA if MJCF lacks color information.

    Returns:
        Path to the exported USD file (absolute path).
    """
    if mj_model is None:
        raise ValueError("`mj_model` must be provided for USD export.")
    if mjcf_root is None:
        raise ValueError("`mjcf_root` is required to reconstruct the MJCF XML for export.")

    target_path = os.path.abspath(save_file_path)
    target_dir = os.path.dirname(target_path)
    if target_dir:
        os.makedirs(target_dir, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="multiverse_mjcf_export_") as tmp_dir:
        xml_path = _write_mjcf_to_file(mjcf_root, tmp_dir)
        importer = MjcfImporter(
            file_path=xml_path,
            fixed_base=fixed_base,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            root_name=root_name,
            inertia_source=inertia_source,
            default_rgba=default_rgba,
            mj_model=mj_model,
        )
        return importer.import_model(save_file_path=target_path)

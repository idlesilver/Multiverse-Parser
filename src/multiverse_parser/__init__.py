import os
import sys

current_dir = os.path.dirname(__file__)
os.environ["PATH"] = os.path.abspath(os.path.join(current_dir, '..', '..', 'ext', 'blender'))
if os.name == 'nt':
    usd_dir = os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'windows', 'lib', 'python'))
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'windows', 'bin'))}"
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'windows', 'lib'))}"
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'windows', 'plugin', 'usd'))}"
else:
    usd_dir = os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'linux', 'lib', 'python'))
    os.environ["PATH"] += f":{os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'linux', 'lib'))}"
    os.environ["PATH"] += f":{os.path.abspath(os.path.join(current_dir, '..', '..', 'USD', 'linux', 'plugin', 'usd'))}"
sys.path.insert(0, usd_dir)

from .importer import UrdfImporter, MjcfImporter, UsdImporter
from .exporter import UrdfExporter, MjcfExporter
from .factory import Factory, Configuration
from .factory import merge_folders
from .factory import InertiaSource
from .factory import (
    WorldBuilder,
    BodyBuilder,
    JointBuilder,
    JointType,
    JointProperty,
    get_joint_axis_and_quat,
    GeomBuilder,
    GeomType,
    GeomProperty,
    MeshBuilder,
    MeshProperty,
    MaterialBuilder,
    MaterialProperty,
)

# from .utils import modify_name, boxify, MjcfBoxify, UrdfBoxify

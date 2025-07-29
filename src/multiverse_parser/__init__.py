import os
import sys
import logging
import tarfile

if os.name == "nt":
    import colorama
    colorama.just_fix_windows_console()

RESET = "\x1b[0m"
COLORS = {
    logging.INFO:     "\x1b[37m",  # white
    logging.WARNING:  "\x1b[33m",  # yellow
    logging.ERROR:    "\x1b[31m",  # red
    logging.CRITICAL: "\x1b[31;1m" # bright/bold red
}

class ColorFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        color = COLORS.get(record.levelno, "")
        message = super().format(record)
        return f"{color}{message}{RESET}"

def configure_logging(level=logging.INFO):
    handler   = logging.StreamHandler(sys.stdout)
    formatter = ColorFormatter("%(asctime)s - %(levelname)-8s - %(message)s")
    handler.setFormatter(formatter)

    root = logging.getLogger()
    root.setLevel(level)
    root.handlers = [handler]           # replace default handler

def extract_tarball():
    here = os.path.dirname(__file__)
    tar_path = os.path.join(here, "external.tar.gz")
    extract_path = os.path.join(here, "external")  # where to extract

    if os.path.exists(tar_path) and not os.path.exists(extract_path):
        print(f"Extracting {tar_path}...")
        with tarfile.open(tar_path, "r:gz") as tar:
            tar.extractall(path=extract_path)

current_dir = os.path.dirname(__file__)
external_parent_dir = os.path.abspath(os.path.join(current_dir, 'external'))
if not os.path.exists(external_parent_dir):
    if os.path.exists(os.path.join(current_dir, 'external.tar.gz')):
        extract_tarball()
    else:
        external_parent_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
os.environ["PATH"] = os.path.abspath(os.path.join(external_parent_dir, 'ext', 'blender'))
if os.name == 'nt':
    usd_dir = os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'lib', 'python'))
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'bin'))}"
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'lib'))}"
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'plugin', 'usd'))}"
else:
    usd_dir = os.path.abspath(os.path.join(external_parent_dir, 'USD', 'linux', 'lib', 'python'))
    os.environ["PATH"] += f":{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'linux', 'lib'))}"
    os.environ["PATH"] += f":{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'linux', 'plugin', 'usd'))}"
sys.path.insert(0, usd_dir)
sys.path = [p for p in sys.path if not p.startswith("/opt/ros")] # Avoid ROS packages interfering, e.g. urdf_parser_py

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

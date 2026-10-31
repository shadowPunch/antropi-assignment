"""Static obstacle geometry read from the world SDF, used to measure ground-truth clearance."""
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory

ROBOT_RADIUS = 0.22  # TurtleBot3 waffle, as in nav2_params.yaml


@dataclass(frozen=True)
class Box:
    name: str
    x: float
    y: float
    yaw: float
    half_x: float
    half_y: float

    def distance(self, px: float, py: float) -> float:
        """Distance from a point to the box surface (0 inside)."""
        dx, dy = px - self.x, py - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return math.hypot(max(abs(lx) - self.half_x, 0.0), max(abs(ly) - self.half_y, 0.0))


@dataclass(frozen=True)
class Cylinder:
    name: str
    x: float
    y: float
    radius: float

    def distance(self, px: float, py: float) -> float:
        return max(math.hypot(px - self.x, py - self.y) - self.radius, 0.0)


def default_world() -> str:
    return os.path.join(get_package_share_directory('tb3_nav_demo'), 'worlds', 'my_world.sdf.xacro')


def load_obstacles(world_xacro: str) -> list:
    """Expand the world xacro and return every static box/cylinder model (ground plane excluded)."""
    sdf = subprocess.run(['xacro', world_xacro], check=True, capture_output=True, text=True).stdout
    shapes = []
    for model in ET.fromstring(sdf).iter('model'):
        geometry = model.find('link/collision/geometry')
        if geometry is None or geometry.find('plane') is not None:
            continue
        x, y, _, _, _, yaw = map(float, model.find('pose').text.split())
        if (box := geometry.find('box')) is not None:
            sx, sy, _ = map(float, box.find('size').text.split())
            shapes.append(Box(model.get('name'), x, y, yaw, sx / 2, sy / 2))
        elif (cyl := geometry.find('cylinder')) is not None:
            shapes.append(Cylinder(model.get('name'), x, y, float(cyl.find('radius').text)))
    return shapes


def gap(shapes: list, x: float, y: float) -> float:
    """Free space between the robot's edge and the nearest obstacle surface (negative = contact)."""
    return min(shape.distance(x, y) for shape in shapes) - ROBOT_RADIUS

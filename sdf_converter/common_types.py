from dataclasses import dataclass
from enum import Enum, auto

class MeshTagMode(Enum):
    default = auto
    manual = auto

def split_pose(pose_str):
    parts = pose_str.split(' ')
    return " ".join(parts[:3]), " ".join(parts[3:])


@dataclass
class Pose:
    xyz: str | None
    rpy: str | None

    @staticmethod
    def extract_from(node):
        xyz, rpy = split_pose(node.text)
        return Pose(xyz, rpy)

    @staticmethod
    def default():
        return Pose("0 0 0", "0 0 0")

    @staticmethod
    def from_vec(translation: list, rotation: list):
        return Pose(" ".join(map(str, translation)), " ".join(map(str, rotation)))

    def to_urdf(self, tag="origin", indent=0):
        return " " * indent + f'<{tag} xyz="{self.xyz}" rpy="{self.rpy}" />'

    def get_translation(self):
        if self.xyz is None:
            return [0, 0, 0]
        else:
            return [float(item) for item in self.xyz.split(" ")]

    def get_rpy(self):
        if self.rpy is None:
            return [0, 0, 0]
        else:
            return [float(item) for item in self.rpy.split(" ")]

class Geometry:
    def open_tag(self):
        return "<geometry>"

    def close_tag(self):
        return "</geometry>"

    @staticmethod
    def extract_from(node, package_with_resource="bringup"):
        assert node.tag == "geometry"
        first = node[0]
        if first.tag == "mesh":
            return Mesh.extract_from(node.find("mesh"), package_with_resource)
        elif first.tag == "cylinder":
            return Cylinder.extract_from(node.find("cylinder"))
        else:
            raise RuntimeError(f"unknown geometry {first.tag}")

    def to_urdf(self, indent=0):
        raise NotImplemented()

@dataclass
class Cylinder(Geometry):
    radius: float
    length: float

    @staticmethod
    def extract_from(node):
        return Cylinder(
            float(node.find("radius").text),
            float(node.find("length").text))

    def to_urdf(self, indent=0):
        lines = []
        prefix = " " * indent
        lines.append(prefix + self.open_tag())
        lines.append(prefix + f'    <cylinder radius="{self.radius}" length="{self.length}" />')
        lines.append(prefix + self.close_tag())
        return "\n".join(lines)

@dataclass
class Mesh(Geometry):
    uri: str
    package_with_resource: str
    tag_manual: str = None
    tag_mode: MeshTagMode = MeshTagMode.default

    def extract_from(node, package_with_resource="bringup"):
        value = node.find("uri").text.replace("model://", "")
        return Mesh(value, package_with_resource)

    def to_urdf(self, indent=0):
        prefix = " " * indent
        lines = []
        lines.append(prefix + self.open_tag())
        if self.tag_mode == MeshTagMode.manual and self.tag_manual is not None:
            lines.append(prefix + f'    ' + self.tag_manual)
        else:
            lines.append(prefix + f'    <mesh filename="package://$(find {self.package_with_resource})/{self.uri}" />')
        lines.append(prefix + self.close_tag())
        return "\n".join(lines)

    def set_manual_tag(self, value):
        self.tag_mode = MeshTagMode.manual
        self.tag_manual = value

@dataclass
class Inertia:
    ixx: float = 0
    ixy: float = 0
    ixz: float = 0
    iyy: float = 0
    iyz: float = 0
    izz: float = 0

    @staticmethod
    def extract_from(node):
        return Inertia(
            ixx=float(node.find("ixx").text),
            ixy=float(node.find("ixy").text),
            ixz=float(node.find("ixz").text),
            iyy=float(node.find("iyy").text),
            iyz=float(node.find("iyz").text),
            izz=float(node.find("izz").text)
        )

    def to_urdf(self, indent=0):
        return " " * indent + f'<inertia ixx="{self.ixx}" ixy="{self.ixy}" ixz="{self.ixz}" iyy="{self.iyy}" iyz="{self.iyz}" izz="{self.izz}" />'

@dataclass
class Inertial:
    pose: Pose | None
    mass: float
    inertia: Inertia

    @staticmethod
    def extract_from(node):
        pose_node = node.find("pose")
        pose = Pose.extract_from(pose_node) if pose_node is not None else None

        return Inertial(
            pose=pose,
            mass=float(node.find('mass').text),
            inertia=Inertia.extract_from(node.find("inertia"))
        )

    def to_urdf(self, indent=0):
        prefix = " " * indent
        outputs = []
        outputs.append(prefix + "<inertial>")
        if self.pose is not None:
            outputs.append(self.pose.to_urdf(tag="origin", indent=indent + 4))
        outputs.append(prefix + f'<mass value="{self.mass}" />')
        outputs.append(self.inertia.to_urdf(indent + 4))
        outputs.append(prefix + "</inertial>")
        return "\n".join(outputs)

@dataclass
class Visual:
    name: str
    geometry: Geometry

    @staticmethod
    def extract_from(node):
        name = node.get("name")
        geometry = Geometry.extract_from(node.find("geometry"))
        return Visual(name, geometry)

    def to_urdf(self, indent=0):
        prefix = " " * indent
        lines = []
        lines.append(prefix + "<visual>")
        lines.append(self.geometry.to_urdf(indent + 4))
        lines.append(prefix + "</visual>")
        return "\n".join(lines)

@dataclass
class Collision:
    name: str
    pose: Pose | None
    geometry: Geometry

    @staticmethod
    def extract_from(node):
        assert node.tag == "collision"
        name = node.get("name")
        pose_node = node.find("pose")
        pose = Pose.extract_from(pose_node) if pose_node is not None else None
        geometry = Geometry.extract_from(node.find("geometry"))
        return Collision(name, pose, geometry)

    def to_urdf(self, indent=0):
        prefix = " " * indent
        lines = []
        lines.append(prefix + "<collision>")
        if self.pose is not None:
            lines.append(self.pose.to_urdf("origin", indent + 4))
        lines.append(self.geometry.to_urdf(indent + 4))
        lines.append(prefix + "</collision>")
        return "\n".join(lines)

@dataclass
class Link:
    name: str
    pose: Pose | None
    inertial: Inertial
    visual: Visual
    collisions: list[Collision]

    @staticmethod
    def extract_from(node):
        assert node.tag == "link"
        name = node.get("name")
        assert name is not None

        pose_node = node.find("pose")
        pose = Pose.extract_from(pose_node) if pose_node is not None else None

        return Link(
            name,
            pose,
            Inertial.extract_from(node.find("inertial")),
            Visual.extract_from(node.find("visual")),
            [Collision.extract_from(item) for item in node.findall("collision")]
        )

    def to_urdf(self, indent=0):
        prefix = " " * indent
        lines = []
        lines.append(prefix + f'<link name="{self.name}">')
        lines.append(self.inertial.to_urdf(indent + 4))
        lines.append(self.visual.to_urdf(indent + 4))
        for collision in self.collisions:
            lines.append(collision.to_urdf(indent + 4))
        lines.append(prefix + "</link>")
        return "\n".join(lines)

@dataclass
class Limit:
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None

    @staticmethod
    def extract_from(node):
        assert node.tag == "limit"
        return Limit(
            lower=float(node.find("lower").text),
            upper=float(node.find("upper").text),
            effort=float(node.find("effort").text),
            velocity=float(node.find("velocity").text)
        )

    def to_urdf(self, indent=0):
        prefix = " " * indent
        return prefix + f'<limit lower="{self.lower}" upper="{self.upper}" effort="{self.effort}" velocity="{self.velocity}" />'

@dataclass
class Axis:
    xyz: str | None
    rpy: str | None
    limit: Limit

    @staticmethod
    def extract_from(node):
        assert node.tag == "axis"
        xyz_node = node.find("xyz")
        xyz = xyz_node.text if xyz_node is not None else None
        rpy_node = node.find("rpy")
        rpy = rpy_node.text if rpy_node is not None else None
        limit = Limit.extract_from(node.find("limit"))
        return Axis(xyz, rpy, limit)

    def to_urdf(self, indent=0):
        """
        In URDF, the axis tag and the limit tag are separate.
        """
        prefix = " " * indent
        lines = []

        match (self.xyz is not None, self.rpy is not None):
            case [True, True]:
                lines.append(prefix + f'<axis xyz="{self.xyz}" rpy="{self.rpy}" />')
            case [True, False]:
                lines.append(prefix + f'<axis xyz="{self.xyz}" />')
            case [False, True]:
                lines.append(prefix + f'<axis rpy="{self.rpy}" />')
            case _:
                pass

        lines.append(prefix + self.limit.to_urdf())
        return "\n".join(lines)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    axis: Axis
    origin: Pose | None

    @staticmethod
    def extract_from(node):
        assert node.tag == "joint"
        name = node.get("name")
        xtype = node.get("type")
        assert name is not None and xtype is not None
        parent = node.find("parent").text
        child = node.find("child").text
        axis = Axis.extract_from(node.find("axis"))
        return Joint(
            name=name,
            type=xtype,
            parent=parent,
            child=child,
            axis=axis,
            origin=None
        )

    def to_urdf(self, indent=0):
        assert self.origin is not None

        prefix = " " * indent
        additional_indent = " " * 4
        lines = []
        lines.append(prefix + f'<joint name="{self.name}" type="{self.type}">')
        lines.append(prefix + additional_indent + f'<parent link="{self.parent}" />')
        lines.append(prefix + additional_indent + f'<child link="{self.child}" />')
        lines.append(self.origin.to_urdf(indent=indent + 4))
        lines.append(self.axis.to_urdf(indent=indent + 4))
        lines.append(prefix + "</joint>")
        return "\n".join(lines)


import xml.etree.ElementTree as ET
from sdf_converter.common_types import *

class TestTypes:
    def test_pose(self):
        node = ET.fromstring("<pose>0 0 .1273 0 -0 0</pose>")
        assert Pose.extract_from(node).to_urdf() == '<origin xyz="0 0 .1273" rpy="0 -0 0" />'

    def test_inertia(self):
        s = """
        <inertia>
          <ixx>0.00610633</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00610633</iyy>
          <iyz>0</iyz>
          <izz>0.01125</izz>
        </inertia>
        """
        node = ET.fromstring(s)
        assert Inertia.extract_from(node).to_urdf() == \
            '<inertia ixx="0.00610633" ixy="0.0" ixz="0.0" iyy="0.00610633" iyz="0.0" izz="0.01125" />'

    def test_inertial(self):
        s = """
          <inertial>
            <mass>4</mass>
            <inertia>
              <ixx>0.00610633</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.00610633</iyy>
              <iyz>0</iyz>
              <izz>0.01125</izz>
            </inertia>
          </inertial>
        """
        node = ET.fromstring(s)
        results = [item.strip() for item in Inertial.extract_from(node).to_urdf().split("\n")]
        assert len(results) == 4
        assert results[0] == "<inertial>"
        assert results[1] == '<mass value="4.0" />'
        assert results[2] == '<inertia ixx="0.00610633" ixy="0.0" ixz="0.0" iyy="0.00610633" iyz="0.0" izz="0.01125" />'
        assert results[3] == '</inertial>'


    def test_geometry_cylinder(self):
        s = """
        <geometry>
          <cylinder>
            <radius>0.075</radius>
            <length>0.038</length>
          </cylinder>
        </geometry>
        """
        node = ET.fromstring(s)
        result = Geometry.extract_from(node).to_urdf()
        lines = [item.strip() for item in result.split("\n")]
        assert len(lines) == 3
        assert lines[0] == "<geometry>"
        assert lines[1] == '<cylinder radius="0.075" length="0.038" />'
        assert lines[2] == "</geometry>"

    def test_geometry_mesh(self):
        s = """
        <geometry>
          <mesh>
            <uri>model://ur10/meshes/base.dae</uri>
          </mesh>
        </geometry>
        """
        node = ET.fromstring(s)
        result = Geometry.extract_from(node).to_urdf()
        lines = [item.strip() for item in result.split("\n")]
        assert len(lines) == 3
        assert lines[0] == "<geometry>"
        assert lines[1] == '<mesh filename="package://$(find bringup)/ur10/meshes/base.dae" />'
        assert lines[2] == "</geometry>"

    def test_visual(self):
        s = """
        <visual name='visual'>
            <geometry>
            <mesh>
            <uri>model://ur10/meshes/shoulder.dae</uri>
            </mesh>
            </geometry>
        </visual>
        """
        node = ET.fromstring(s)
        result = Visual.extract_from(node).to_urdf()
        lines = [item.strip() for item in result.split("\n")]

        assert lines[0] == "<visual>"
        assert lines[1] == "<geometry>"
        assert lines[2] == '<mesh filename="package://$(find bringup)/ur10/meshes/shoulder.dae" />'
        assert lines[3] == "</geometry>"
        assert lines[4] == "</visual>"
        assert len(lines) == 5

    def test_collision(self):
        s = """
          <collision name='collision3'>
            <pose>0 0.001 0.5735 1.5707 0 0</pose>
            <geometry>
              <cylinder>
                <radius>0.045</radius>
                <length>0.119</length>
              </cylinder>
            </geometry>
          </collision>
        """
        node = ET.fromstring(s)
        result = Collision.extract_from(node).to_urdf()
        lines = [item.strip() for item in result.split("\n")]
        assert lines[0] == "<collision>"
        assert lines[1] == '<origin xyz="0 0.001 0.5735" rpy="1.5707 0 0" />'
        assert lines[2] == "<geometry>"
        assert lines[3] == '<cylinder radius="0.045" length="0.119" />'
        assert lines[4] == "</geometry>"
        assert lines[5] == "</collision>"
        assert len(lines) == 6


    def test_link(self):
        s = """
            <link name='wrist_1'>
              <pose>1.1843 0.049041 0.1273 3.14159 3.58979e-09 3.14159</pose>
              <inertial>
                <mass>1.96</mass>
                <inertia>
                  <ixx>0.01</ixx>
                  <ixy>0.02</ixy>
                  <ixz>0.03</ixz>
                  <iyy>0.04</iyy>
                  <iyz>0.05</iyz>
                  <izz>0.06</izz>
                </inertia>
              </inertial>
              <collision name='collision'>
                <pose>0 0.115 0.0025 0 0 0</pose>
                <geometry>
                  <cylinder>
                    <radius>0.0455</radius>
                    <length>0.119</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <mesh>
                    <uri>model://ur10/meshes/wrist1.dae</uri>
                  </mesh>
                </geometry>
              </visual>
            </link>
        """
        node = ET.fromstring(s)
        link_obj = Link.extract_from(node)
        assert link_obj.pose.xyz == "1.1843 0.049041 0.1273"
        assert link_obj.pose.rpy == "3.14159 3.58979e-09 3.14159"
        result = link_obj.to_urdf()
        lines = [item.strip() for item in result.split("\n")]
        assert lines == [
            '<link name="wrist_1">',
            '<inertial>',
            '<mass value="1.96" />',
            '<inertia ixx="0.01" ixy="0.02" ixz="0.03" iyy="0.04" iyz="0.05" izz="0.06" />',
            '</inertial>',
            '<visual>',
            '<geometry>',
            '<mesh filename="package://$(find bringup)/ur10/meshes/wrist1.dae" />',
            '</geometry>',
            '</visual>',
            '<collision>',
            '<origin xyz="0 0.115 0.0025" rpy="0 0 0" />',
            '<geometry>',
            '<cylinder radius="0.0455" length="0.119" />',
            '</geometry>',
            '</collision>',
            '</link>'
        ]

    def test_axis(self):
        s = """
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <lower>-6.28319</lower>
              <upper>6.28319</upper>
              <effort>330</effort>
              <velocity>2.16</velocity>
            </limit>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
        """
        node = ET.fromstring(s)
        axis = Axis.extract_from(node)
        lines = [item.strip() for item in axis.to_urdf().split("\n")]
        assert lines == [
            '<axis xyz="0 0 1" />',
            '<limit lower="-6.28319" upper="6.28319" effort="330.0" velocity="2.16" />'
        ]


    def test_joint(self):
        s = """
        <joint name='shoulder_pan' type='revolute'>
          <child>shoulder</child>
          <parent>base</parent>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <lower>-6.28319</lower>
              <upper>6.28319</upper>
              <effort>330</effort>
              <velocity>2.16</velocity>
            </limit>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
        </joint>
        """
        node = ET.fromstring(s)
        joint = Joint.extract_from(node)
        joint.origin = Pose.default()
        result = joint.to_urdf()
        lines = [item.strip() for item in result.split("\n")]
        assert lines == [
            '<joint name="shoulder_pan" type="revolute">',
            '<parent link="base" />',
            '<child link="shoulder" />',
            '<origin xyz="0 0 0" rpy="0 0 0" />',
            '<axis xyz="0 0 1" />',
            '<limit lower="-6.28319" upper="6.28319" effort="330.0" velocity="2.16" />',
            '</joint>'
        ]
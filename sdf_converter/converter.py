from sdf_converter.common_types import *
from sdf_converter.transformation import compute_relative_pose
import argparse
import xml.etree.ElementTree as ET

def main(input_file, model_dir, output_file):
    root = ET.parse(input_file).getroot()
    model = root.find("model")
    robot_name = model.get("name")

    links = []
    joints = []
    order_info = []
    name_to_link = {}

    for k, child in enumerate(model):
        match child.tag:
            case "link":
                link = Link.extract_from(child)
                links.append(link)
                name_to_link[link.name] = link
                order_info.append("link")
            case "joint":
                joints.append(Joint.extract_from(child))
                order_info.append("joint")
            case _:
                raise RuntimeError(f"Unexpected tag. {child.tag}")

    # Set the origin field in the Joint object
    for joint in joints:
        parent_link = name_to_link[joint.parent]
        child_link = name_to_link[joint.child]

        joint.origin = compute_relative_pose(parent_link, child_link)

    # Set the model path
    for link in links:
        geometry = link.visual.geometry

        if isinstance(geometry, Mesh):
            if model_dir is None:
                geometry.set_manual_tag(f'<mesh filename="package://{geometry.uri}" />')
            else:
                geometry.set_manual_tag(f'<mesh filename="package://{model_dir}/{geometry.uri}" />')


    BEGIN = f"""\
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot_name}">
    """

    END = "</robot>"

    output = []
    output.append(BEGIN)

    idx_link = 0
    idx_joint = 0
    indent = 4
    # Add a robot footprint link
    # output.append(' ' * indent + '<link name="robot_footprint"></link>')
    for item in order_info:
        match item:
            case "link":
                output.append(links[idx_link].to_urdf(indent))
                idx_link += 1
            case "joint":
                output.append(joints[idx_joint].to_urdf(indent))
                idx_joint += 1
            case _:
                raise RuntimeError("Unexpected tag in the order info.")

    output.append(END)

    result = "\n".join(output)

    if output_file:
        with open(output_file, 'w') as f:
            f.write(result)
            f.write("\n")
    else:
        print(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='sdf2urdf_converter',
        description='Converts a sdf file to a urdf file.',
        epilog='Text at the bottom of help')

    parser.add_argument('filename')
    parser.add_argument('-o', '--output', action="store", default=None, help="output file path")
    parser.add_argument("-d", "--model-dir", action="store", default=None, help="The model directory")

    args = parser.parse_args()
    assert args.filename.endswith(".sdf")

    input_file = args.filename
    model_dir = args.model_dir if args.model_dir is not None else None
    main(input_file, model_dir, args.output)

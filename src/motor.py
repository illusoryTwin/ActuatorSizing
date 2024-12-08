import toml
from lxml import etree
import numpy as np


class Motor:
    def __init__(self, name, gear_ratio, torque_limit, inertia, mass):
        self.name = name
        self.gear_ratio = gear_ratio
        self.torque_limit = torque_limit
        self.inertia = inertia
        self.mass = mass
        self._set_ctrlrange()
        self.calculate_inertia_reflected()

    def _set_ctrlrange(self):
        self.low_torque_limit = 0
        self.ctrlrange = [self.low_torque_limit, self.torque_limit]

    def calculate_inertia_reflected(self):
        self.reflected_inertia = self.gear_ratio**2 * self.inertia

    def calculate_motor_inertia_tensor(self, joint_rotation_axis, offset_distance):

        motor_inertia_tensor = np.zeros((3, 3))
        axis_idx = {"x": 0, "y": 1, "z": 2}[joint_rotation_axis]
        motor_inertia_tensor[axis_idx, axis_idx] = self.inertia

        offset_distance_mag2 = np.dot(offset_distance, offset_distance)
        parallel_axis_correction = self.mass * offset_distance_mag2
        parallel_axis_tensor = np.eye(3) * parallel_axis_correction

        adjusted_motor_inertia_tensor = motor_inertia_tensor + parallel_axis_tensor
        return adjusted_motor_inertia_tensor

    def __repr__(self):
        return f"Motor(name={self.name}, gear_ratio={self.gear_ratio}, torque_limit={self.torque_limit})"


def _load_motor_config(toml_file):
    """Loads motor configurations from a TOML file."""
    config = toml.load(toml_file)
    motors = {}

    for joint_name, motor_data in config["motors"].items():
        motor = Motor(
            name=motor_data["name"],
            gear_ratio=motor_data["gear_ratio"],
            torque_limit=motor_data["max_allowable_torque"],
            inertia=motor_data["inertia"],
            mass=motor_data["mass"],
        )
        motors[joint_name] = motor

    return motors


def _get_joint_axis_idx(axes_string: str) -> int:

    joint_axis_mapping = {"x": 0, "y": 1, "z": 2}

    components = axes_string.split()
    components = [int(x) for x in components]
    if components == [1, 0, 0]:
        axis_name = "x"
    elif components == [0, 1, 0]:
        axis_name = "y"
    elif components == [0, 0, 1]:
        axis_name = "z"

    return joint_axis_mapping.get(
        axis_name, -1
    )  # Return -1 if the axis name is 'error'


def _get_links_lengths(xml_file_path, motors_config_file):
    tree = etree.parse(xml_file_path)

    root = tree.getroot()
    motors = _load_motor_config(motors_config_file)
    links_sizes = {}

    for joint_name, motor in motors.items():
        joint = root.find(f".//joint[@name='{joint_name}']")
        rotation_axis = joint.get("axis")

        axis_components = [int(x) for x in rotation_axis.split()]
        print(axis_components)

        if axis_components == [1, 0, 0]:
            axis_idx = 0
        elif axis_components == [0, 1, 0]:
            axis_idx = 1
        elif axis_components == [0, 0, 1]:
            axis_idx = 2
        else:
            axis_idx = 100000000

        print("axis_idx", axis_idx)

        if joint is not None:
            body = joint.getparent()

            link_size_str = body.get("pos")
            link_size_components = [float(x) for x in link_size_str.split()]
            link_size = link_size_components[axis_idx]

            link_name = body.get("name")

            if link_size is not None and link_name is not None:
                # In mujoco pos is defined for the half of the body, so double the values by 2 to get the correct sizes
                link_length = float(link_size) * 2
                links_sizes[link_name] = link_length

    return links_sizes


def update_motors_xml(
    xml_file_path, motors_config_file, updated_xml_file="updated_robot_model.xml"
):

    links_lenghts = _get_links_lengths(xml_file_path, motors_config_file)
    tree = etree.parse(xml_file_path)

    root = tree.getroot()
    motors = _load_motor_config(motors_config_file)

    # Find or create an actuator element in the root
    actuators = root.find(".//actuator")
    if actuators is None:
        actuators = etree.SubElement(root, "actuator")

    for joint_name, motor in motors.items():
        joint = root.find(f".//joint[@name='{joint_name}']")
        if joint is not None:
            # Extract the axis from the joint
            axes_elements = joint.get("axis")
            rotat_joint_axis_idx = _get_joint_axis_idx(axes_elements)

            body = joint.getparent()

            inertial = body.find("inertial")

            if inertial is not None:

                current_mass_str = inertial.get("mass")
                if current_mass_str is not None:
                    current_mass = float(current_mass_str)
                    updated_mass = current_mass + motor.mass
                    inertial.set("mass", str(updated_mass))
                else:
                    inertial.set("mass", motor.mass)

                current_link_diaginertia_str = inertial.get("diaginertia")
                current_link_diaginertia_components = [
                    float(x) for x in current_link_diaginertia_str.split()
                ]

                # Update the component corresponding to the rotational joint axis
                current_diaginertia_component = current_link_diaginertia_components[
                    rotat_joint_axis_idx
                ]
                updated_diaginertia_component = (
                    current_diaginertia_component + motor.inertia
                )
                current_link_diaginertia_components[rotat_joint_axis_idx] = (
                    updated_diaginertia_component
                )

                # Convert the updated list back to a string
                upd_link_diaginertia_components_str = " ".join(
                    f"{x:.6g}" for x in current_link_diaginertia_components
                )
                inertial.set("diaginertia", upd_link_diaginertia_components_str)

            motor_element = etree.SubElement(actuators, "motor")
            motor_element.set("name", f"{motor.name}_{joint_name}")  # Motor name
            motor_element.set("joint", joint_name)
            motor_element.set(
                "ctrlrange", f"{motor.ctrlrange[0]} {motor.ctrlrange[1]}"
            )  # Associated joint

            joint.set("armature", str(motor.reflected_inertia))
            print(f"Set armature for {joint_name}: {motor.reflected_inertia}")

    # Save the updated XML
    tree.write(
        updated_xml_file, pretty_print=True, xml_declaration=True, encoding="UTF-8"
    )

    print(f"Updated XML saved to {updated_xml_file}")

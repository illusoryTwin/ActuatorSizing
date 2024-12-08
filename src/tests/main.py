from lxml import etree

file_path="../mjmodel.xml"
tree = etree.parse(file_path)

root = tree.getroot()

joints = root.findall('.//joint')

for joint in joints:
    print(joint.get('name'), joint.get('axis'))


reflected_inertias = {
    "shoulder_pan_joint": 4,
    "shoulder_lift_joint" : 4,
    "elbow_joint": 1,
    "wrist_1_joint": 2,
    "wrist_2_joint" : 1,
    "wrist_3_joint": 3
}

masses =  {
    "shoulder_pan_joint": 4,
    "shoulder_lift_joint" : 4,
    "elbow_joint": 1,
    "wrist_1_joint": 2,
    "wrist_2_joint" : 1,
    "wrist_3_joint": 3
}

new_inertias = {
    "shoulder_pan_joint": "1 1 1",
    "shoulder_lift_joint" : "4 4 4",
    "elbow_joint": "3 3 3",
    "wrist_1_joint": "2 2 2",
    "wrist_2_joint" : "1 1 1",
    "wrist_3_joint": "5 5 5"
}

# Motor configuration for each joint
motor_params = {
    "shoulder_pan_joint": {"ctrlrange": "0 1"},
    "shoulder_lift_joint": {"ctrlrange": "0 1"},
    "elbow_joint": {"ctrlrange": "0 1"},
    "wrist_1_joint": {"ctrlrange": "0 1"},
    "wrist_2_joint": {"ctrlrange": "0 1"},
    "wrist_3_joint": {"ctrlrange": "0 1"},
}


for joint_name, inertia_val in reflected_inertias.items():
    joint = root.find(f".//joint[@name='{joint_name}']")
    if joint is not None:
        joint.set("armature", str(inertia_val))
        print(joint)

for joint_name, mass in masses.items():
    joint = root.find(f".//joint[@name='{joint_name}']")
    if joint is not None:
        body = joint.getparent()  # Assuming the joint's parent is the body
        inertial = body.find("inertial")
        if inertial is not None:
            inertial.set("mass", str(mass))


for joint_name, new_inertia in new_inertias.items():
    joint = root.find(f".//joint[@name='{joint_name}']")
    if joint is not None:
        body = joint.getparent()  # Assuming the joint's parent is the body
        inertial = body.find("inertial")
        if inertial is not None:
            print("new_inertia", new_inertia)
            inertial.set("diaginertia", str(new_inertia))

# Find or create an actuator element in the root
actuators = root.find(".//actuator")
if actuators is None:
    actuators = etree.SubElement(root, "actuator")


for joint_name, motor_param in motor_params.items():
    joint = root.find(f".//joint[@name='{joint_name}']")
    if joint is not None:
        motor = etree.SubElement(actuators, "motor")
        motor.set("name", f"motor_{joint_name}")  # Motor name
        motor.set("joint", joint_name)  # Associated joint
        motor.set("ctrlrange", motor_param["ctrlrange"])  # Control range


# Save the updated XML to a new file
output_file = "updated_model_mass.xml"
tree.write(output_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")

from motor import load_motor_config, update_motors_xml

if __name__ == "__main__":
    motors_config_file1 = "../config/2_model_config.toml"
    motors = load_motor_config(motors_config_file1)
    xml_file_path = "../mjmodel.xml"
    updated_filename = "../models/test.xml"
    update_motors_xml(xml_file_path, motors_config_file1, updated_filename)

from _model_parser import update_motors_xml

BASELINE_MODEL_PATH = "../mjmodel.xml"

def main(robot_config_id: int):
    motors_config_file = f"../config/{robot_config_id}_model_config.toml"
    xml_file_path = BASELINE_MODEL_PATH
    updated_file_path = f"../models/{robot_config_id}_robot_model.xml"
    update_motors_xml(xml_file_path, motors_config_file, updated_file_path)

if __name__ == "__main__":
    num_models = 4
    for i in range(num_models):
        main(robot_config_id=i+1)

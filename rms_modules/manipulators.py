
import json
import os.path

class VXManipulator():
    def __init__(self, category):
        """
        Initialize the class instance with the specified parameters.
        """

        self.robot_model, self.host, self.user, self.password = load_config_json(f"{category}.json")






def load_config_json(name):
    """
    Load the configuration parameters from the coresponding JSON file. 

    @param name : The name of the JSON file in the config subdir.

    @return robot_model : The model of the manipulator i.e., vx250 or vx300s.
    @return host : The host address of the machine controlling the manipulator.
    @return username : The username of the machine controlling the manipulator.
    @return password : The password of the machine controlling the manipulator.
    """

    path = os.path.join(os.path.expanduser("~/rms_ros/src/rms_ros/config"), name)
    with open(path, "r") as file:
        data = json.load(file)
    robot_model = data["robot_model"]
    host = data["host"]
    username = data["username"]
    password = data["password"]
    return robot_model, host, username, password
    

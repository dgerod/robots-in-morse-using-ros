from morse_helpers.storage import FileStorage
from morse.builder.actuators import Armature


class AdeptViperS650(Armature):
    _name = "OAT Viper s650"
    _short_desc = "Omron Adept - 6DoF Robotic Arm"

    def __init__(self, name=None):
        model_file = FileStorage().find("oat_viper_s650.blend", False)
        Armature.__init__(self, name, model_name=model_file)
        self.create_ik_targets(["joint_6"])

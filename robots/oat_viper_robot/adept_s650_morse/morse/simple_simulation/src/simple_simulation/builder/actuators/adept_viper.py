from morse.builder.actuators import Armature


class AdeptViperS650(Armature):
    _name = "OAT Viper s650"
    _short_desc = "Omron Adept - 6DoF Robotic Arm"

    def __init__(self, name=None):
        Armature.__init__(self, name, model_name="oat_viper_s650")
        self.create_ik_targets(["joint_6"])

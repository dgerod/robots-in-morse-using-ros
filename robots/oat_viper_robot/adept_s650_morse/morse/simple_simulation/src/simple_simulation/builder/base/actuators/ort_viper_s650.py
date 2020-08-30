from morse.builder.actuators import Armature
from morse_helpers.storage import FileStorage


class OrtViperS650(Armature):
    _name = "ORT Viper s650"
    _short_desc = "Omron Robotics - 6DoF Robotic Arm"

    def __init__(self, name=None):

        ONLY_LOCAL_SEARCH = False
        file_ = FileStorage().find("ort_viper_s650.blend", ONLY_LOCAL_SEARCH)

        Armature.__init__(self, name, model_name=file_)
        self.create_ik_targets(["joint_6"])

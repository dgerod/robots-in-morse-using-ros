import os.path
from morse_helpers import morse_local_config as settings


class FileStorage:

    def __init__(self):
        self._load()

    def find(self, file_name, global_search=True):

        file_name = self._add_extension(file_name)

        absolute_file_path = self._find_robot_components(file_name)
        if absolute_file_path is None:
            absolute_file_path = self._find_environment(file_name)
            if absolute_file_path is None:
                absolute_file_path = self._find_object(file_name)
                if absolute_file_path is None and global_search is True:
                    absolute_file_path = file_name

        return absolute_file_path

    def _find_robot_components(self, file_name):

        directory_path = self._models_directory + "robots/"
        if os.path.isfile(directory_path + file_name):
            file_path = directory_path + file_name
        else:
            directory_path = self._models_directory + "actuators/"
            if os.path.isfile(directory_path + file_name):
                file_path = directory_path + file_name
            else:
                directory_path = self._models_directory + "sensors/"
                if os.path.isfile(directory_path + file_name):
                    file_path = directory_path + file_name
                else:
                    file_path = None

        return file_path

    def _find_environment(self, file_name):

        directory_path = self._environments_directory
        if os.path.isfile(directory_path + file_name):
            file_path = self._environments_directory + file_name
        else:
            file_path = None

        return file_path

    def _find_object(self, file_name):

        directory_path = self._objects_directory
        if os.path.isfile(directory_path + file_name):
            file_path = self._objects_directory + file_name
        else:
            file_path = None

        return file_path

    def _load(self):
        self._simulation_path = settings._simulation_path
        self._models_directory = settings._models_directory

        self._environments_directory = settings._environments_directory
        self._objects_directory = settings._objects_directory

    @staticmethod
    def _add_extension(file_name):
        if file_name.endswith(".blend"):
            file_name = file_name
        else:
            file_name = file_name + ".blend"
        return file_name
import os.path
from morse_helpers import morse_local_config as settings


class FileStorage:

    def __init__(self):
        self._load()

    def find(self, file_name):
        
        if file_name.endswith(".blend"):
            file_name = file_name
        else:
            file_name = file_name + ".blend"
                       
        directory_path = self._simulation_path + "data/" + self._environments_directory
        if not os.path.isfile(directory_path + file_name):

            directory_path = self._simulation_path + "data/" + self._objects_directory
            if not os.path.isfile(directory_path + file_name):
                abs_file_path = file_name
            else:
                abs_file_path = self._objects_directory + file_name

        else:
            abs_file_path = self._environments_directory + file_name

        return abs_file_path

    def _load(self):
        self._simulation_path = settings._simulation_path
        self._environments_directory = settings._environments_directory
        self._objects_directory = settings._objects_directory
import json
from morse_helpers import morse_local_config as simulation_settings


class SimulationLocalSettings:

    def __init__(self):
        self._load()

    def __str__(self):
        return str(self._settings)

    def get(self):
        return self._settings

    def show_info(self):

        print("\n---------------------------------------------")
        print("-         SIMULATION LOCAL SETTINGS         -")
        print("---------------------------------------------")
        print("simulation ")
        print("  + name: ", self._settings['simulation']['name'])
        print("  + path: ", self._settings['simulation']['path'])
        print("  + script: ", self._settings['simulation']['script'])
        print("models ")
        print("  + root: ", self._settings['models'])
        print("  + environments: ", self._settings['environments'])
        print("  + objects: ", self._settings['objects'])
        print("middleware ")

        idx = 1
        for location in self._settings['middleware']:
            print("  + location %d" % idx)
            print("    - directory: ", location[0])
            print("    - class_path: ", location[1])
            idx += 1

        print("---------------------------------------------\n")

    def _load(self):

        self._settings = {
            'simulation': {
                'name': simulation_settings.get_settings().simulation_name,
                'path': simulation_settings.get_settings().simulation_path,
                'script': simulation_settings.get_settings().simulation_script
            },
            'models': simulation_settings.get_settings().models_directory,
            'environments': simulation_settings.get_settings().environments_directory,
            'objects': simulation_settings.get_settings().objects_directory,
            'middleware': simulation_settings.get_settings().middleware_locations,
        }

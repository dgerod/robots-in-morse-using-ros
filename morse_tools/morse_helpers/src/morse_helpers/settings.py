import json
import morse_helpers.morse_local_config as settings


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
                'name': settings._simulation_name,
                'path': settings._simulation_path,
                'script': settings._simulation_script
            },
            'models': settings._models_directory,
            'environments': settings._environments_directory,
            'objects': settings._objects_directory,
            'middleware': settings._middleware_locations,
        }

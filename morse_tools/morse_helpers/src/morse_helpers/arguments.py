import json
import os.path
from morse_helpers import morse_local_config as simulation_settings


class SimulationLocalArguments:

    def __init__(self):
        self._load()

    def __str__(self):
        #return json.dumps(self._arguments, indent=2, sort_keys=True)
        return str(self._arguments)

    def get(self):
        return self._arguments

    def _load(self):

        file_path = simulation_settings._simulation_path + "arguments.json"

        self._arguments = None
        if os.path.isfile(file_path):
            with open(file_path, 'r') as f:
                self._arguments = json.load(f)
        return self._arguments

import json
import os, rospkg

# Exported module information
# ---------------------------------------------

_simulation_name = ""
_simulation_path = ""
_environments_directory = ""
_objects_directory = ""
_middleware_locations = []


def _prepare_simulation_basic_information(file_path):

    # Configure with the name of the experiment.
    simulation_path = os.path.dirname(os.path.abspath(file_path))
    simulation_name = os.path.basename(simulation_path)
    simulation_path = simulation_path + "/"

    return simulation_path, simulation_name


def _prepare_model_data(simulation_name):

    environments_directory = simulation_name + '/environments/'
    objects_directory = simulation_name + '/props/'

    return environments_directory, objects_directory


def _prepare_middleware_data(simulation_path, simulation_name):

    # Middleware location preparation
    # Remember middleware objects must be added to "__init__.py" of the directory.

    mw_locations = []

    # Middleware location based local simulation
    directory = "/" + simulation_name + '/runtime/middleware/'
    class_path = directory[1:].replace("/", ".")
    mw_locations.append([directory, class_path])

    # Middleware location based on morse_ros packag
    rospack = rospkg.RosPack()
    directory = rospack.get_path('morse_ros') + '/src/morse_ros/middleware/'
    class_path = directory[1:].replace("/", ".")
    class_path = "morse_ros.middleware."
    mw_locations.append([directory, class_path])

    return mw_locations


def load(file_path):

    global _simulation_path, _simulation_name
    global _environments_directory, _objects_directory
    global _middleware_locations

    _simulation_path, _simulation_name = _prepare_simulation_basic_information(file_path)
    _environments_directory, _objects_directory = _prepare_model_data(_simulation_name)
    _middleware_locations = _prepare_middleware_data(_simulation_path, _simulation_name)

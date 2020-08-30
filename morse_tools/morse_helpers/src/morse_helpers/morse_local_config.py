import os
import rospkg


class Settings:
    def __init__(self):
        self.simulation_name = ""
        self.simulation_path = ""
        self.simulation_script = ""
        self.models_directory = ""
        self.environments_directory = ""
        self.objects_directory = ""
        self.middleware_locations = []


_settings = Settings()


def load(file_path):

    global _settings

    _settings.simulation_name, _settings.simulation_path, _settings.simulation_script = \
        _prepare_simulation_basic_information(file_path)
    _settings.models_directory, _settings.environments_directory, _settings.objects_directory = \
        _prepare_model_data(_settings.simulation_path, _settings.simulation_name)
    _settings.middleware_locations = \
        _prepare_middleware_data(_settings.simulation_path, _settings.simulation_name)


def get_settings():
    return _settings


def _prepare_simulation_basic_information(file_path):

    # Configure with the name of the experiment.

    simulation_name = os.path.basename(os.path.dirname(os.path.abspath(file_path)))
    simulation_path = os.path.dirname(os.path.abspath(file_path)) + '/'
    simulation_script = os.path.basename(os.path.abspath(file_path))

    return simulation_name, simulation_path, simulation_script


def _prepare_model_data(simulation_path, simulation_name):

    models_directory = simulation_path + 'data/' + simulation_name + '/'
    environments_directory = models_directory + 'environments/'
    objects_directory = models_directory + 'props/'

    return models_directory, environments_directory, objects_directory


def _prepare_middleware_data(simulation_path, simulation_name):

    # Middleware location preparation
    # Remember middleware objects must be added to "__init__.py" of the directory.

    mw_locations = []

    # Middleware location based local simulation
    local_path = 'runtime/middleware/'
    directory = simulation_path + local_path
    class_path = ("/" + simulation_name + "/" + local_path)[1:].replace('/', '.')
    mw_locations.append([directory, class_path])

    # Middleware location based on morse_ros package
    rospack = rospkg.RosPack()
    directory = rospack.get_path('morse_ros') + '/src/morse_ros/middleware/'
    class_path = directory[1:].replace('/', '.')
    # TBD-FIX @dgerod: Obtain correctly the path using rospkg
    class_path = 'morse_ros.middleware.'
    mw_locations.append([directory, class_path])

    return mw_locations


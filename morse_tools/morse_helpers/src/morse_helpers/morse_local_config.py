# =============================================================================
# Local configuration of the simulation.
# dgerod@xyz-lab.org.es
# =============================================================================

import os, rospkg

# Exported module information
# ---------------------------------------------

simulation_name = ""
simulation_path = ""
environment_dir = "";
objects_dir = "";
mw_dir = "";
mw_loc = "";

class LocalSettings:
    pass

# Read and prepare information needed by the simulation
# ---------------------------------------------

def _prep_middleware_data():

    # Middleware location based on directory.
    # Remember middleware objects must be added to "__init__.py" of the directory.    
    rospack = rospkg.RosPack()
    mw_dir = rospack.get_path('morse_ros') + '/src/morse_ros/middleware/'
    mw_loc = mw_dir[1:].replace("/", ".")
    return mw_dir, mw_loc

def configure_simulation(filePath):
    
    global simulation_name, simulation_path
    global objects_dir, environment_dir
    global mw_dir, mw_loc
    
    # Configure with the name of the experiment.
    simulation_path =  os.path.dirname(os.path.abspath(filePath))
    simulation_name = os.path.basename(simulation_path)
    simulation_path = simulation_path + "/"

    # Mount structure of directories used by the simulation.
    objects_dir = simulation_name + '/props/'
    environment_dir = simulation_name + '/environments/'
    
    # Middleware location based on directory.
    mw_dir, mw_loc = _prep_middleware_data()
    mw_loc = "morse_ros.middleware."

    _show_info()
# ---------------------------------------------

def _show_info():

    global simulation_name, simulation_path
    global objects_dir, environment_dir
    global mw_dir, mw_loc
 
    print("---")
    print("+ Simulation")
    print("  - Name: ", simulation_name)
    print("  - Path: ", simulation_path)
    print("+ Environments: ", environment_dir)
    print("+ Objects: ", objects_dir)
    print("+ Middleware ")
    print("  - Path: ", mw_dir)
    print("  - Location: ", mw_loc)
    print("---")
    
# =============================================================================

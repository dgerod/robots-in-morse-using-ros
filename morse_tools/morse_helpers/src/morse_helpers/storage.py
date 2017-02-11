# =============================================================================
# Management of the files used in the simulation.
# dgerod@xyz-lab.org.es
# =============================================================================

import os.path
from morse_helpers import morse_local_config as settings

class FileStorage:
    _simulation_name = settings.simulation_name
    _simulation_path = settings.simulation_path
    _env_dir = settings.environment_dir;
    _object_dir = settings.objects_dir; 

    @staticmethod
    def find(FileName):  
        
        if FileName.endswith(".blend"):
            file_name = FileName
        else:
            file_name = FileName + ".blend"   
                       
        dir_path = FileStorage._simulation_path + "data/" + FileStorage._env_dir
        print(dir_path)
        if os.path.isfile(dir_path + file_name) == False:
            dir_path = FileStorage._simulation_path + "data/" + FileStorage._object_dir
            print(dir_path)
            if os.path.isfile(dir_path + file_name) == False:
                FilePath = file_name
            else:
                FilePath = FileStorage._object_dir + file_name;
        else:
            FilePath = FileStorage._env_dir + file_name;    
        
        print(FilePath)
        return FilePath
  
# =============================================================================


## How to use helpers
#### Configure helpers in your simulation environment

TBD

#### Use them in your simulation script

The helpers will be access in your "default.py" file as:

    from morse_helpers import morse_local_config as local_settings
    local_settings.configure_simulation(__file__)
    from morse_helpers.storage import FileStorage
    from morse_helpers.adapters import ROSRegister

Similar to your own defined robots, sensors and actuators:
    
    from your_simulation.builder.sensors import YourSensor

And the will be used in your "default.py" file as:

    from morse_helpers import morse_local_config as local_settings
    local_settings.configure_simulation(__file__)
    from morse_helpers.storage import FileStorage
    from morse_helpers.adapters import ROSRegister
    
    from morse.builder import FakeRobot
    from your_simulation.builder.sensors import YourSensor

    vcam = FakeRobot()
    vcam.name = "TopCamera"
    topCam = YourSensor()
    topCam.properties(tag="box")
    topCam.properties(relative=True)        
    vcam.append(topCam)
    vcam.translate(x=-0.25, z=2.0)
    vcam.rotate(y=pi/2)    
    
    ROSRegister.add_topic(topCam, "/topCam")
    
    box = PassiveObject(FileStorage.find("objects"), 'Box') 
    env = Environment(FileStorage.find("empty_world_2"))    

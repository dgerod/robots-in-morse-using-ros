#! /usr/bin/env morseexec
# =======================================================================================
# Experiment using MORSE 
# =======================================================================================

import simple_simulation.helpers.morse_local_config as exp_settings
from simple_simulation.helpers import adapters

from builder import environment
from morse.builder import Environment
from morse.core.morse_time import TimeStrategies

from morse.builder import FakeRobot
from morse.builder.actuators import KukaLWR
from morse.builder.sensors import ArmaturePose

# ---------------------------------------------------------------------------------------
        
def create_simulation():
    
    # Create the robot
    # ----------------------------------------------------------

    robot = FakeRobot() 
    robot.name = "kuka_lwr"

    # Add robot arm to the robot 
    arm = KukaLWR()    
    arm_pose = ArmaturePose()
    arm.append(arm_pose)        
    robot.append(arm)    
    
    # Set-up ROS connection 
    # ----------------------------------------------------------
    
    topic_base_name = "/" + robot.name + "/"
    robot.add_default_interface('ros')
    
    # Arm - follow_joint_trajectory + joint_state ---
    
    adapters.register_ros_topic(arm_pose, 
                                name = ("klwr/joint_states"),
                                topic_class = 'JointStatePublisher' )
     
    adapters.register_ros_action(arm, 
                                 name = ("klwr"),
                                 action_class = 'ArmControllerByActions' )
    
    # Environment 
    # ----------------------------------------------------------
    
    env = Environment( exp_settings.environment_dir + "empty_world.blend")
    env.set_camera_location([10.0, -10.0, 10.0])
    env.set_camera_rotation([1.0470, 0, 0.7854])

    env.show_framerate(True)
    
# ---------------------------------------------------------------------------------------

create_simulation()

# =======================================================================================

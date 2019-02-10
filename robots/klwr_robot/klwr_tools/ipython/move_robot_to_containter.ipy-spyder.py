# %% Create a ROS node
# -------------------------------------------------------------

import rospy
from geometry_msgs.msg import Pose
rospy.init_node("test_robot")

#
# import pymorse2
# simulator = pymorse.Morse()
# simulator.rpc('simulation','get_scene_objects')
# how to add a object.

# %% List objects in morse
# -------------------------------------------------------------

import pymorse

simulator = pymorse.Morse()
objects simulator.rpc('simulation','get_scene_objects')

# %% Initialize connections with robot
# -------------------------------------------------------------
# [!!!] The robot fake in listener mode must be running.

from fake_robot.client import listener
from fake_robot.client import commander

mode_manager = listener.InteractionModeManager()

camera_request = listener.Camera()
arm_request = listener.Arm()
gripper_request = listener.Gripper()

#gripper_commander = commander.GripperService()

camera_request.add_object("v1", "videotape", [0,0,0,0,0,0])
camera_request.add_object("v2", "videotape", [0,0,0,0,0,0])
camera_request.add_object("v3", "videotape", [0,0,0,0,0,0])
camera_request.add_object("v4", "videotape", [0,0,0,0,0,0])
camera_request.set_expected_status(True)

mode_manager.change_to_interactive_mode()

# %% Prepare connection to KB and PS
# -------------------------------------------------------------
# [!!!] The KB and PS nodes of ROSPlan must be running.

from rosplan.controller import knowledge_base as kb
from rosplan.controller import planning_system as ps


kb.init()
kb.clear_all()

# %% Prepare connection to KB and PS
# -------------------------------------------------------------
# [!!!] The KB and PS nodes of ROSPlan must be running.

from rosplan.controller import knowledge_base as kb
from rosplan.controller import planning_system as ps

kb.init()
kb.clear_predicates()
kb.clear_goals()

ps.init()


# %% Prepare initial scene data
# -------------------------------------------------------------

item = kb.Item("c1", "container")
kb.add_instance(item)

item = kb.Item("b1", "box", Pose())
kb.add_instance(item)

kb.add_instance("v1", "videotape", Pose())
kb.add_instance("v2", "videotape", Pose())
kb.add_instance("v3", "videotape", Pose())
kb.add_instance("v4", "videotape", Pose())

kb.add_predicate('robot-at', p='b1')
kb.add_predicate('free-tool')
kb.add_predicate('object-at', o='v1', p='c1')
kb.add_predicate('object-at', o='v2', p='c1')
kb.add_predicate('object-at', o='v3', p='c1')
kb.add_predicate('object-at', o='v4', p='c1')

# %% Set goals
# -------------------------------------------------------------

kb.add_goal('object-at', o='v1', p='b1')
kb.add_goal('object-at', o='v2', p='b1')
kb.add_goal('object-at', o='v3', p='b1')
kb.add_goal('object-at', o='v4', p='b1')

# %% And execute the plan
# -------------------------------------------------------------
# [!!!] The KB node of ROSPlan must be running.

ps.plan()

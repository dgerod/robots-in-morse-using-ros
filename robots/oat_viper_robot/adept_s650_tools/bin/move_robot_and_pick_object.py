import math

import rospy
import tf, moveit_commander
from geometry_msgs.msg import PoseStamped


def move_to(group, pose, frame_id="/map"):

    #???
    #end_effector_link = group.get_end_effector_link()
    #initial_pose = group.get_current_pose().pose
    #initial_position = group.get_current_joint_values()

    rot = [pose[3], pose[4], pose[5]]
    q = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2], 'rzyx')

    target_pose = PoseStamped()
    target_pose.pose.position.x = pose[0]
    target_pose.pose.position.y = pose[1]
    target_pose.pose.position.z = pose[2]
    target_pose.pose.orientation.x = q[0]
    target_pose.pose.orientation.y = q[1]
    target_pose.pose.orientation.z = q[2]
    target_pose.pose.orientation.w = q[3]
    target_pose.pose.orientation.w = q[3]
    target_pose.header.frame_id = frame_id

    group.set_pose_target(target_pose)

    plan = group.plan()
    group.execute(plan, wait=True)
    rospy.sleep(1)

    group.clear_pose_targets()


def move_to_home(group):

    group.clear_pose_targets()
    group.set_named_target("home")

    plan = group.plan()
    group.execute(plan, wait=True)
    rospy.sleep(1)

    group.clear_pose_targets()


def main():
    moveit_commander.roscpp_initialize('move_and_grab')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    move_to_home(group)

    pose = [0.5, 0, 0.12, 0, math.pi, 0]
    move_to(group, pose)

    moveit_commander.roscpp_shutdown()


if __name__ == "main":
    main()
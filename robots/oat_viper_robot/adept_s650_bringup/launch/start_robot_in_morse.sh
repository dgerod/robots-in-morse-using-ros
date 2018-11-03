#!/usr/bin/env bash
# We are assuming the simulation in adept_s650_more package is already register
# in .morse/config, so an entry like following one exists there:
#
#   oat_viper_robot = YOUR_CATKIN_ENVIRONMENT/adept_s650_morse/morse/simple_simulation
#

morse run oat_viper_robot default.py
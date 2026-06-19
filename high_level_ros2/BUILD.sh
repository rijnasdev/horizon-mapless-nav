#!/bin/bash

xacro urdf/rover.urdf.xacro > urdf/rover.urdf
colcon build

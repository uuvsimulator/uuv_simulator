#!/bin/sh
DESCRIPTION_PKG=`rospack find orion_7p_description`
ORION_7P_DAE=$DESCRIPTION_PKG/openrave/orion_7p_standalone.dae

openrave-robot.py $ORION_7P_DAE --info links

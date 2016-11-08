#!/bin/sh
# This script opens the OpenRAVE visualizer
DESCRIPTION_PKG=`rospack find orion_7p_description`
ORION_7P_DAE=$DESCRIPTION_PKG/openrave/orion_7p_standalone.dae

openrave $ORION_7P_DAE

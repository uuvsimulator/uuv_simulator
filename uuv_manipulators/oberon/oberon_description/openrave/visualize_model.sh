#!/bin/sh
# This script opens the OpenRAVE visualizer
DESCRIPTION_PKG=`rospack find oberon_description`
OBERON_DAE=$DESCRIPTION_PKG/openrave/oberon_standalone.dae

openrave $OBERON_DAE

#!/bin/sh
DESCRIPTION_PKG=`rospack find oberon_description`
OBERON_DAE=$DESCRIPTION_PKG/openrave/oberon_standalone.dae

openrave-robot.py $OBERON_DAE --info links

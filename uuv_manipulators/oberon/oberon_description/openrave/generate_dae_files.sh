#!/bin/sh
#~ This script generates the .dae and .urdf files for all the robots
#~ found in the oberon_description/robots

DESCRIPTION_PKG=`rospack find oberon_description`
FILES=$(find $DESCRIPTION_PKG/robots -type f -name oberon_standalone.xacro)

for XACRO in $FILES
do
  FILENAME=$(basename "$XACRO")
  FILENAME="${FILENAME%.*}"
  URDF=$DESCRIPTION_PKG/robots/$FILENAME.urdf
  DAE=$DESCRIPTION_PKG/openrave/$FILENAME.dae
  rosrun xacro xacro $XACRO > $URDF
  rosrun collada_urdf urdf_to_collada $URDF $DAE
  echo "URDF successfully generated for [$FILENAME.xacro]"
  #~ rosrun moveit_ikfast round_collada_numbers.py $DAE $DAE 5 > /dev/null
done

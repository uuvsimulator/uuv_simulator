set(dirs
    rexrov
)

# Root path for the ROS applications
set(ROS_SHARE ${ROS_ROOT}/..)
# Set the path to the xacro script to convert to XACRO files to URDF
set(ROS_XACRO ${ROS_SHARE}/xacro/xacro.py)

foreach(dir ${dirs})
    add_subdirectory(${dir})
endforeach()

cmake_minimum_required(VERSION 2.8.3)
project(uuv_descriptions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_descriptions
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Install python scripts
catkin_install_python(PROGRAMS scripts/spawn_model
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch meshes robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T
      test/test_urdf_files.test
      )
     add_rostest(${T})
  endforeach()
endif()

find_package(gazebo REQUIRED)

set (worlds
  underwater_heightmap.world
  empty_underwater.world
  ocean_waves.world
)

# Install all the world files
install(FILES ${worlds}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds)

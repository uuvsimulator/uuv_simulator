set(MODEL_NAME sand_heightmap)

set (files
  model.sdf
  model.config
  heightmap.dae
  soil_sand_0045_01.jpg
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})

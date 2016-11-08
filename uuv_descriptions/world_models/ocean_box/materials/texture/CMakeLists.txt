set (files 
  soil_sand_0045_01.jpg
  water_water_0076_03_s.jpg
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME}/materials/textures/)

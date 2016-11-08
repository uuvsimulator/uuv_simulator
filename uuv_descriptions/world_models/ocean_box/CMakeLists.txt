set(MODEL_NAME ocean_box)

add_subdirectory(materials/scripts)
add_subdirectory(materials/texture)

set (files
  model.sdf
  model.config
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})

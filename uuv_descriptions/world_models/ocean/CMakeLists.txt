set(MODEL_NAME ocean)

install(DIRECTORY materials
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME}
        PATTERN "*~" EXCLUDE)

set (files
  model.sdf
  model.config
  mesh.dae
  mesh_below.dae
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})

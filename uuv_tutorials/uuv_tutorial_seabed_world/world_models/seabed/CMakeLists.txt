set(MODEL_NAME seabed)

set (files
  model.sdf
  model.config
  seabed.stl
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})

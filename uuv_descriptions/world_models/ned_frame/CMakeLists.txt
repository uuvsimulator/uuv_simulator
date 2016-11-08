set(MODEL_NAME ned_frame)
set (files
  model.sdf
  model.config
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})

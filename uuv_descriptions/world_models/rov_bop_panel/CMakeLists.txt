set(MODEL_NAME rov_bop_panel)

set (files
  model.sdf
  model.config
  BOP_Panel.dae
  SmallValve.dae
  Valve.dae
  metal_bare_0045_01.jpg
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})

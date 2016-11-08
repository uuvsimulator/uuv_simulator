set(dirs
    ned_frame
    ocean
    sand_heightmap
    ocean_box
    rov_bop_panel
    umbilical
)

foreach(dir ${dirs})
  add_subdirectory(${dir})
endforeach()

set(dirs
  seabed
)

foreach(dir ${dirs})
  add_subdirectory(${dir})
endforeach()

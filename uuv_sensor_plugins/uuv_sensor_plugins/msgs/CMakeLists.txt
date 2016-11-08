find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  SensorDvl.proto
  SensorMagnetic.proto
  SensorImu.proto
  SensorPressure.proto
  SensorRpt.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(uuv_sensors_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_sensors_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})

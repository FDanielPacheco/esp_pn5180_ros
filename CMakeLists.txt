cmake_minimum_required(VERSION 3.0.2)
project(esp_pn5180_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
)

# Find libserialposix library (Improved)
find_library(LIB_SERIAL NAMES libserialposix libserialposix.a libserialposix.h )

if(LIB_SERIAL)
  message(STATUS "Found libserialposix library: ${LIB_SERIAL}")
else()
  message(FATAL_ERROR "Could not find libserialposix library!  Check path and name.")
endif()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Content.msg
  Destination.msg
  Dimensions.msg 
  Vehicle.msg 
  FirmwareVersion.msg
  SetTimeout.msg 
  TagMrths.msg 
  TagCargo.msg 
  RequestMrths.msg
  RequestCargo.msg 
  RequestSetTimeout.msg
  RequestSerial.msg
  RequestStop.msg 
  RequestVersion.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_generation roscpp std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}  # This is usually correct
  /usr/local/include
  /usr/local/lib
)

# Remove link_directories.  find_library() handles this.
# link_directories(
#  /usr/local/lib
# )

add_library( ${PROJECT_NAME}  # Consider renaming to avoid conflicts
  include/${PROJECT_NAME}/simpleJson.h
  src/simpleJson.cpp 
)

## The recommended prefix ensures that target names across packages don't collide
add_executable( rfid_supervisor_node
  src/firmware.cpp
  src/simpleJson.cpp
)

add_dependencies( rfid_supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
if(LIB_SERIAL) # Only link if the library was found
  target_link_libraries(rfid_supervisor_node
    ${catkin_LIBRARIES}
    ${LIB_SERIAL}
  )
else()
  message(WARNING "libserialposix library not found. Skipping linking.")
endif()
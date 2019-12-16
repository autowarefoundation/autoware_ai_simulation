cmake_minimum_required(VERSION 2.8.3)
project(wf_simulator)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_health_checker
  autoware_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2
)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(wf_simulator_SRC
  src/wf_simulator_core.cpp
  src/vehicle_model/wfsim_model_interface.cpp
  src/vehicle_model/wfsim_model_ideal.cpp
  src/vehicle_model/wfsim_model_constant_acceleration.cpp
  src/vehicle_model/wfsim_model_time_delay.cpp
)
add_executable(wf_simulator src/wf_simulator_node.cpp ${wf_simulator_SRC})
target_link_libraries(wf_simulator ${catkin_LIBRARIES})

install(TARGETS wf_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(
  PROGRAMS scripts/fitParamDelayInputModel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-wf_simulator
    test/test_wf_simulator.test
    test/src/test_wf_simulator.cpp
    src/wf_simulator_core.cpp
    ${wf_simulator_SRC}
  )
  add_dependencies(test-wf_simulator ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-wf_simulator ${catkin_LIBRARIES})
endif()

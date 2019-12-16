cmake_minimum_required(VERSION 2.8.3)
project(lgsvl_simulator_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  roslib
  rospy
  rqt_gui
  rqt_gui_py
  sensor_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(programs)
  install(TARGETS ${programs}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

foreach(dir launch rviz config media resource)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach()

install(PROGRAMS scripts/launch_sim.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(PYTHON_SCRIPTS src/lgsvl_simulator_bridge/__init__.py src/lgsvl_simulator_bridge/configurator.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

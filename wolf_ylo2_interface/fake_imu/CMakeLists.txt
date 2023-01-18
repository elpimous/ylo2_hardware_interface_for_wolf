cmake_minimum_required(VERSION 2.8.3)
project(fake_imu)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(fake_imu src/fake_imu.cpp)
add_dependencies(fake_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_imu ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(TARGETS fake_imu
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

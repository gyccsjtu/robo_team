cmake_minimum_required(VERSION 3.0.2)
project(robocup_team_launch)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE)

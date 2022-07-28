cmake_minimum_required(VERSION 2.8.3)
project(land_rect_center)
find_package(catkin REQUIRED COMPONENTS
  mavros
  message_generation
  roscpp
  rospy
  std_msgs
)
add_message_files(FILES center.msg) # 这句话要注意添加
generate_messages(DEPENDENCIES std_msgs) # 这句话要注意添加
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(landing_rect_node src/landing_rect.cpp) # 这句话要注意添加
# 这句话要注意添加
target_link_libraries(landing_rect_node 
${catkin_LIBRARIES}
)



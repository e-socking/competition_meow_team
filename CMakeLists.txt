cmake_minimum_required(VERSION 3.8)
project(team_meow_challenge)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(pose_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()




add_executable(armor_node src/armor_vision_node.cpp)
add_executable(sphere_node src/sphere_vision_node.cpp)
add_executable(rect_node src/tanglegal.cpp)

#测试：
add_executable(armor_subscriber src/接受测试.cpp)
ament_target_dependencies(armor_subscriber
  rclcpp
  referee_pkg
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  armor_subscriber
  DESTINATION lib/${PROJECT_NAME}
)




ament_target_dependencies(armor_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
ament_target_dependencies(rect_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
ament_target_dependencies(sphere_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(armor_node ${OpenCV_LIBS})
target_link_libraries(sphere_node ${OpenCV_LIBS})
target_link_libraries(rect_node ${OpenCV_LIBS})
# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(armor_node ${typesupport_target})
target_link_libraries(sphere_node ${typesupport_target})
target_link_libraries(rect_node ${typesupport_target})
# 安装Vision_test
install(TARGETS armor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS sphere_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS rect_node
  DESTINATION lib/${PROJECT_NAME}
)
# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(TestNode src/test.cpp)
ament_target_dependencies(TestNode
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(TestNode ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(TestNode ${typesupport_target})

# 安装Vision_test
install(TARGETS TestNode
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件夹到 ROS 2 标准共享目录
install(DIRECTORY
  launch  # 你的 launch 文件夹（对应截图中项目里的 launch 目录）
  DESTINATION share/${PROJECT_NAME}  # 目标路径：install/share/功能包名/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
ament_package()

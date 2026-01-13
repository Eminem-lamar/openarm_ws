####################################################################################################
#                                         PROJECT INFORMATION                                      #
####################################################################################################
cmake_minimum_required(VERSION 3.5)
# 强制指定MuJoCo 3.4.0的绝对路径（请确认路径与你的实际安装路径一致）
set(MUJOCO_INCLUDE_DIR "/home/zjc/.mujoco/mujoco340/include")
set(MUJOCO_LIB_DIR "/home/zjc/.mujoco/mujoco340/bin")

# 手动添加头文件路径（优先级最高，解决找不到mujoco.h的问题）
include_directories(
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # 保留包自身的include路径
)

# 手动添加库文件路径
link_directories(${MUJOCO_LIB_DIR})

# 【修正后的路径验证】正确判断mujoco.h是否存在并打印
if(EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(STATUS "✅ Found mujoco.h at: ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
else()
  message(WARNING "❌ Could NOT find mujoco.h at: ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
endif()
project(mujoco_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

####################################################################################################
#                                          DEPENDENCIES                                            #
####################################################################################################
find_package(ament_cmake REQUIRED)                                                                  # So ROS2 can compile?
find_package(glfw3 REQUIRED)                                                                        # Graphics Library FrameWork
find_package(rclcpp REQUIRED)                                                                       # ROS2 C++ libraries
find_package(sensor_msgs REQUIRED)                                                                  # sensor_msgs::msg::JointState
find_package(std_msgs REQUIRED)                                                                     # std_msgs::msg::Float64MultiArray

# MuJoCo
set(MUJOCO_PATH "/home/zjc/.mujoco/mujoco340") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(include)                                                                        # Local header files
include_directories(${MUJOCO_PATH}/include/mujoco)                                                  # MuJoCo header files
include_directories(${MUJOCO_PATH}/include)                                                         # Additional MuJoCo header files
link_directories(${MUJOCO_PATH}/lib)                                                                # Location of MuJoCo libraries

####################################################################################################
#                                           EXECUTABLES                                            #
####################################################################################################
add_executable(mujoco_node src/nodes/mujoco_node.cpp
                           src/mujoco_ros.cpp)                                                      # Define new executable, list source files
    
target_link_libraries(mujoco_node mujoco glfw)                                                      # Link GLFW libraries to the executable
                                     
ament_target_dependencies(mujoco_node rclcpp sensor_msgs std_msgs)                                  # List ROS2 dependencies for this executable


####################################################################################################
#                                           INSTALLATION                                           #
####################################################################################################

install(TARGETS mujoco_node
        DESTINATION lib/${PROJECT_NAME})                                                            # Install so ROS2 can find it

install(DIRECTORY launch
                  model
        DESTINATION share/${PROJECT_NAME})                                                          # Install so ROS2 can find them
        
ament_package()

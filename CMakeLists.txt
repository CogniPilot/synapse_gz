cmake_minimum_required(VERSION 3.8)
project(synapse_gz)


set(CMAKE_EXPORT_COMPILE_COMMANDS)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(synapse_protobuf REQUIRED)
find_package(synapse_tinyframe REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/clients/gz_client.cpp
  src/clients/tcp_client.cpp
)
ament_target_dependencies(${PROJECT_NAME} synapse_tinyframe synapse_protobuf gz-transport13 Boost rclcpp)
target_include_directories(${PROJECT_NAME} PRIVATE src/clients)

#==========================================================
# install
#==========================================================

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



#==========================================================
# testing
#==========================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

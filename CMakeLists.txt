cmake_minimum_required(VERSION 3.8)
project(synapse_gz)


set(CMAKE_EXPORT_COMPILE_COMMANDS)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(synapse_protobuf REQUIRED)
find_package(synapse_tinyframe REQUIRED)
find_package(hcdformat REQUIRED)
find_package(gz-transport12 REQUIRED)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME} synapse_tinyframe synapse_protobuf gz-transport12)

#==========================================================
# install
#==========================================================

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
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

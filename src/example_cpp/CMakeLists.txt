cmake_minimum_required(VERSION 3.8)
project(example_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp)
install(TARGETS
	node_01
	DESTINATION lib/${PROJECT_NAME}
)

add_executable(node_03 src/node_03.cpp)
ament_target_dependencies(node_03 rclcpp)
install(TARGETS
  node_03
  DESTINATION lib/${PROJECT_NAME}
)

# node: topic_publisher_01
string(ASCII 27 Esc) # Esc \033 八进制的27即Esc
set(Green "${Esc}[0;32m")   #Green
find_library(ADD_LIB fadd_win64 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
message("${Green}Found dynamic link library success: ${ADD_LIB}")
find_library(SIMODEL_LIB simodel_win64 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
message("${Green}Found dynamic link library success: ${SIMODEL_LIB}")

include_directories(
	./src/posix/
	./src/fadd/
	./include/example_cpp/
)
add_executable(topic_publisher_01 
	src/topic_publisher_01.cpp
	src/posix/spi.cpp
	# src/fadd/fadd.c
	)

ament_target_dependencies(topic_publisher_01 rclcpp std_msgs)

target_link_libraries(topic_publisher_01 ${ADD_LIB} ${SIMODEL_LIB})
# target_link_libraries(topic_publisher_01 ${ADD_LIB} fadd_win64)
# target_link_libraries(topic_publisher_01 ${SIMODEL_LIB} simodel_win64)

install(TARGETS
  topic_publisher_01
  DESTINATION lib/${PROJECT_NAME}
)

# node: subscribe_01
add_executable(topic_subscribe_01 src/topic_subscribe_01.cpp)
ament_target_dependencies(topic_subscribe_01 rclcpp std_msgs)
install(TARGETS
  topic_subscribe_01
  DESTINATION lib/${PROJECT_NAME}
)


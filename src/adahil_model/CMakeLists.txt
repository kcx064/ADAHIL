cmake_minimum_required(VERSION 3.8)
project(adahil_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adahil_interface REQUIRED)

string(ASCII 27 Esc) # Esc \033 八进制的27即Esc
set(Green "${Esc}[0;32m")   #Green
find_library(SIMODEL_LIB simodel_win64 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
message("${Green}Found dynamic link library success: ${SIMODEL_LIB}")

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include/adahil_model/
)

add_executable(sim_model src/sim_model.cpp)
target_include_directories(sim_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sim_model PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sim_model
  "rclcpp"
  "adahil_interface"
)
target_link_libraries(sim_model ${SIMODEL_LIB})

install(TARGETS sim_model
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})


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

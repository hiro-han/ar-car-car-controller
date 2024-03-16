cmake_minimum_required(VERSION 3.8)
project(ar_car_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ar_car_info REQUIRED)

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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

set(TARGET ar_car_controller)
add_executable(${TARGET}
  src/main.cpp
  src/ArCarController.cpp
  src/SerialConnection.cpp
)
ament_target_dependencies(${TARGET} "rclcpp")
ament_target_dependencies(${TARGET} "ar_car_info")
install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})

get_target_property(target_options ${TARGET} COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")



set(TEST_TARGET test_serial)
add_executable(${TEST_TARGET}
  test/main.cpp
  test/SerialTest.cpp
  test/SerialConnection.cpp
)
ament_target_dependencies(${TEST_TARGET} "rclcpp")
ament_target_dependencies(${TEST_TARGET} "ar_car_info")
install(TARGETS ${TEST_TARGET} DESTINATION lib/${PROJECT_NAME})

get_target_property(target_options ${TEST_TARGET} COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

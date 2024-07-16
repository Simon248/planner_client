cmake_minimum_required(VERSION 3.8)
project(planning_client_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_FIND_DEBUG_MODE 1)

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# tesseract
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_task_composer REQUIRED)
# tesseract_ros2
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

message(STATUS "TESSERACT_COMPILE_OPTIONS: ${TESSERACT_COMPILE_OPTIONS}")
message(STATUS "TESSERACT_CLANG_TIDY_ARGS: ${TESSERACT_CLANG_TIDY_ARGS}")
message(STATUS "TESSERACT_CXX_VERSION: ${TESSERACT_CXX_VERSION}")

message("TARTEDVUEZDTVKQYZDVHUCDVQGDVKGQDKQDV")
message(${PROJECT_NAME})

# target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
# target_include_directories(${PROJECT_NAME} PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>"
# )


add_executable(${PROJECT_NAME}_node src/planning_client.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES} ${tesseract_common_LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>")





# target_compile_options(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION ${TESSERACT_CXX_VERSION})
# target_include_directories(${PROJECT_NAME}_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")

# install(
#     TARGETS
#       ${PROJECT_NAME}_node
#      #  ${PROJECT_NAME}_raster_example
#     RUNTIME DESTINATION lib/${PROJECT_NAME}
#    )


# add_executable(planning_client src/planning_client.cpp)

# include_directories(
#   # ${tesseract_command_language_INCLUDE_DIRS}
#   ${tesseract_common_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
# )



# target_include_directories(planning_client PUBLIC
  
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(planning_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS planning_client
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright a${rclcpp_INCLUDE_DIRS}
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()




# ###############
# cmake_minimum_required(VERSION 3.8)
# project(planning_client_pkg)

# # find dependencies
# # ROS
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_ros REQUIRED)
# # tesseract
# find_package(tesseract_command_language REQUIRED)
# find_package(tesseract_common REQUIRED)
# find_package(tesseract_motion_planners REQUIRED)
# find_package(tesseract_task_composer REQUIRED)
# # tesseract_ros2
# find_package(tesseract_monitoring REQUIRED)
# find_package(tesseract_msgs REQUIRED)
# find_package(tesseract_rosutils REQUIRED)

# # Include directories
# include_directories(
#   ${tesseract_command_language_INCLUDE_DIRS}
#   ${tesseract_common_INCLUDE_DIRS}
#   ${tesseract_planning_INCLUDE_DIRS}
#   ${tesseract_msgs_INCLUDE_DIRS}
#   ${tesseract_rosutils_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
# )

# # Add compile options
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Add executable
# add_executable(planning_client src/planning_client.cpp)

# # Link libraries

# ###############
# cmake_minimum_required(VERSION 3.8)
# project(planning_client_pkg)

# # find dependencies
# # ROS
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_ros REQUIRED)
# # tesseract
# find_package(tesseract_command_language REQUIRED)
# find_package(tesseract_common REQUIRED)
# find_package(tesseract_motion_planners REQUIRED)
# find_package(tesseract_task_composer REQUIRED)
# # tesseract_ros2
# find_package(tesseract_monitoring REQUIRED)
# find_package(tesseract_msgs REQUIRED)
# find_package(tesseract_rosutils REQUIRED)

# # Include directories
# include_directories(
#   ${tesseract_command_language_INCLUDE_DIRS}
#   ${tesseract_common_INCLUDE_DIRS}
#   ${tesseract_planning_INCLUDE_DIRS}
#   ${tesseract_msgs_INCLUDE_DIRS}
#   ${tesseract_rosutils_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
# )

# # Add compile options
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Add executable
# add_executable(planning_client src/planning_client.cpp)

# # Link libraries
# target_link_libraries(planning_client
#   ${tesseract_common_LIBRARIES}
#   ${tesseract_environment_LIBRARIES}
#   ${tesseract_planning_LIBRARIES}
#   ${tesseract_msgs_LIBRARIES}
#   ${tesseract_rosutils_LIBRARIES}
#   ${rclcpp_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
# )

# # Include directories for target
# target_include_directories(planning_client PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # Set compile features
# target_compile_features(planning_client PUBLIC c_std_99 cxx_std_17)

# # Install targets
# install(TARGETS planning_client
#   DESTINATION lib/${PROJECT_NAME})

# # Build testing
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
# target_link_libraries(planning_client
#   ${tesseract_common_LIBRARIES}
#   ${tesseract_environment_LIBRARIES}
#   ${tesseract_planning_LIBRARIES}
#   ${tesseract_msgs_LIBRARIES}
#   ${tesseract_rosutils_LIBRARIES}
#   ${rclcpp_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
# )

# # Include directories for target
# target_include_directories(planning_client PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # Set compile features
# target_compile_features(planning_client PUBLIC c_std_99 cxx_std_17)

# # Install targets
# install(TARGETS planning_client
#   DESTINATION lib/${PROJECT_NAME})

# # Build testing
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
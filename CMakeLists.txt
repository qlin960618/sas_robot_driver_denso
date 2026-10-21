cmake_minimum_required(VERSION 3.8)
project(sas_robot_driver_denso)

# Modifications Copyright (c) 2026 Hung-Ching Huang (qlin1806@g.ecc.u-tokyo.ac.jp)
# 2026-10-21: Build/install the ROS 2 node.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)

##### CPP LIBRARY #####

add_library(${PROJECT_NAME} SHARED
    src/sas_driver_bcap.cpp
    src/sas_robot_driver_denso.cpp
    src/bcap/bCAPClient/bcap_client.c
    src/bcap/bcap_common.c
    src/bcap/dn_com.c
    src/bcap/dn_common.c
    src/bcap/dn_device.c
    src/bcap/dn_tcp.c
    src/bcap/dn_udp.c
    src/bcap/dn_thread.c
    src/bcap/dn_socket.c
    )

ament_target_dependencies(${PROJECT_NAME} Eigen3 sas_core)
target_link_libraries(${PROJECT_NAME} dqrobotics)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bcap>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bcap/bCAPClient>
    )

target_compile_definitions(${PROJECT_NAME} PRIVATE -D_USE_LINUX_API)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 sas_core)

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

##END## CPP LIBRARY #####

add_executable(sas_robot_driver_denso_node src/sas_robot_driver_denso_node.cpp)
ament_target_dependencies(sas_robot_driver_denso_node rclcpp rcl_interfaces sas_common sas_core sas_robot_driver)
target_link_libraries(sas_robot_driver_denso_node ${PROJECT_NAME})
target_compile_features(sas_robot_driver_denso_node PRIVATE cxx_std_17)
install(TARGETS sas_robot_driver_denso_node DESTINATION lib/${PROJECT_NAME})
install(FILES LICENSE LICENSE.GPL-3.0 LICENSE.bcap DESTINATION share/${PROJECT_NAME})

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

cmake_minimum_required(VERSION 3.8)
project(ros_robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)

    set(_pytest_tests test/test_pep257.py)
    foreach(test_path ${_pytest_tests})
        get_filename_component(test_name ${test_path} NAME_WE)
        ament_add_pytest_test(
            ${test_name}
            ${test_path}
            APPEND_ENV
            PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
            TIMEOUT
            60
            WORKING_DIRECTORY
            ${CMAKE_SOURCE_DIR}
        )
    endforeach()
endif()

ament_package()
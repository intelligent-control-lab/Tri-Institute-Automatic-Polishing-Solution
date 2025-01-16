cmake_minimum_required(VERSION 3.12.0)
project(RobotController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# debug setting
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# add definition
add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)


find_package(OsqpEigen REQUIRED)
find_package(LibXml2 REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories(${LIBXML2_INCLUDE_DIR})
include_directories ( "${EIGEN3_INCLUDE_DIR}" )


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware_interface/  
    ${CMAKE_CURRENT_SOURCE_DIR}/include/function/  
    ${CMAKE_CURRENT_SOURCE_DIR}/include/controller/ 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/force_torque_sensor/ 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model/ 
)

# /* -------------------------------------------------------------------------- */
# /*                         Robot-Specific Dependencies                        */
# /* -------------------------------------------------------------------------- */

file(GLOB POS_CONTROL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/position_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_interface/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/force_torque_sensor/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_model/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/function/*.cpp
    )

add_executable(PositinerController ${POS_CONTROL_SRC})

target_link_libraries(PositinerController PRIVATE
    pthread
    rt
    m
    OsqpEigen::OsqpEigen
    )

target_link_libraries(PositinerController PRIVATE ${LIBXML2_LIBRARIES})

file(GLOB FORCE_CONTROL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/force_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_interface/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/force_torque_sensor/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_model/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/function/*.cpp
    )

add_executable(ForceController ${FORCE_CONTROL_SRC})

target_link_libraries(ForceController PRIVATE
    pthread
    rt
    m
    OsqpEigen::OsqpEigen
    )

target_link_libraries(ForceController PRIVATE ${LIBXML2_LIBRARIES})

file(GLOB TEST_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_interface/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/force_torque_sensor/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_model/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/function/*.cpp
    )

add_executable(test ${TEST_SRC})

target_link_libraries(test PRIVATE
    pthread
    rt
    m
    OsqpEigen::OsqpEigen
    )

target_link_libraries(test PRIVATE ${LIBXML2_LIBRARIES})

# file(GLOB RECORD_SRC
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/record_position.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_interface/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/force_torque_sensor/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_model/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/function/*.cpp
#     )

# add_executable(record_position ${RECORD_SRC})

# target_link_libraries(record_position PRIVATE
#     pthread
#     rt
#     m
#     OsqpEigen::OsqpEigen
#     )

# target_link_libraries(record_position PRIVATE ${LIBXML2_LIBRARIES})

# file(GLOB IK_SRC
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/IK.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_interface/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/force_torque_sensor/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_model/*.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/function/*.cpp
#     )

# add_executable(IK ${IK_SRC})

# target_link_libraries(IK PRIVATE
#     pthread
#     rt
#     m
#     OsqpEigen::OsqpEigen
#     )

# target_link_libraries(IK PRIVATE ${LIBXML2_LIBRARIES})
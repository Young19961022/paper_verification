cmake_minimum_required(VERSION 3.5)
project(paper_solver)
add_definitions(-std=c++11)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${PCL_DEFINITIONS})
aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}
                                      ${OpenCV_LIBS})
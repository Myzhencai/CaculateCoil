cmake_minimum_required(VERSION 3.1)
project(cppsolver)

# 支持gdb调试
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED COMPONENTS SuiteSparse EigenSparse Multithreading LAPACK)
include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})
message("current path is :${PROJECT_SOURCE_DIR}")

# add_executable(mag_models mag_models.cpp)
# target_link_libraries(mag_models PRIVATE ${CERES_LIBRARIES})
# add_executable(mag_models file_utils.cpp)
# target_link_libraries(mag_models PRIVATE ${CERES_LIBRARIES})

# add_executable(mag_models globalcost.cpp file_utils.cpp  mag_models.cpp utils.cpp )
# target_link_libraries(mag_models PRIVATE ${CERES_LIBRARIES})

add_executable(mag_models generatetestdata.cpp file_utils.cpp  mag_models.cpp utils.cpp )
target_link_libraries(mag_models PRIVATE ${CERES_LIBRARIES})

# add_executable(mag_models mag-inverse.cpp file_utils.cpp  mag_models.cpp utils.cpp )
# target_link_libraries(mag_models PRIVATE ${CERES_LIBRARIES})
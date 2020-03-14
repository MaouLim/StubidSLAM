cmake_minimum_required(VERSION 3.15)
project(StubidSLAM)

set(CMAKE_CXX_STANDARD 14)

set(EIGEN3_INCLUDE_DIRS deps/Eigen3/include/eigen3/)
set(Sophus_INCLUDE_DIRS deps/Sophus/include/)
set(OpenCV_DIR "D:/OpenCV/4.1.2/build_with_contrib_mingw5.4.0/")

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Test1 test/test1.cpp)
target_link_libraries(Test1 ${OpenCV_LIBS})


add_executable(RUN_VO src/run_ov.cpp)
target_link_libraries(RUN_VO ${OpenCV_LIBS})
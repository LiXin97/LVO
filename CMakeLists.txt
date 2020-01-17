cmake_minimum_required(VERSION 3.14)
project(LVO)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE Release)

find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        include/
)

file(GLOB SOURCEFILES src/*.cc include/*.hpp src/*/*cc )

add_library(LVO_lib SHARED ${SOURCEFILES})

target_link_libraries(LVO_lib
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        )


add_executable(LVO_EuRoc app/EuRoc.cc)
target_link_libraries(LVO_EuRoc LVO_lib ${CMAKE_THREAD_LIBS_INIT})


#TEST
add_executable(test_FM test/test_feature_manger.cc)
target_link_libraries(test_FM LVO_lib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_stereo_frame test/test_stereo_frame.cc)
target_link_libraries(test_stereo_frame LVO_lib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_mono_extract test/test_mono_extract.cc)
target_link_libraries(test_mono_extract LVO_lib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_stereo_tri test/test_stereo_tri.cc)
target_link_libraries(test_stereo_tri LVO_lib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_tuple test/test_tuple.cc)
target_link_libraries(test_tuple ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_tri_angle test/test_tri_angle.cc)
target_link_libraries(test_tri_angle LVO_lib ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_tracking test/test_tracking.cc)
target_link_libraries(test_tracking LVO_lib ${CMAKE_THREAD_LIBS_INIT})
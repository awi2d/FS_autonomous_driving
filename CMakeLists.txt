cmake_minimum_required(VERSION 3.19)
project(state_estimator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O3")

set(EIGEN_DIR "C:/Users/Idefix/CLionProjects/eigen")
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please run \"git clone https://gitlab.com/libeigen/eigen.git\" or set the directory path above to the directory you ran the command in.")
endif()

set(OpenCV_DIR "C:/Users/Idefix/CLionProjects/opencv/my_build/install")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(OpenCV REQUIRED)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_dnn)
message("include Directories: ${OpenCV_INCLUDE_DIRS}")
include_directories(${EIGEN_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(state_estimator src/main.cpp src/velocity_estimator.cpp src/VisualPipeline.cpp src/VisualPipeline.h src/StateEstimator.cpp src/StateEstimator.h)
target_link_libraries(state_estimator ${OpenCV_LIBS})
#https://stackoverflow.com/questions/35984678/use-opencv-with-clion-ide-on-windows
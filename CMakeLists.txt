message("start executing CMakeList.txt of state_estimator")
cmake_minimum_required(VERSION 3.19)
project(state_estimator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O3")

SET(CMAKE_MODULE_PATH "C:/src/eigen/cmake" "${CMAKE_SOURCE_DIR}/cmake-modules/" "${CMAKE_SOURCE_DIR}/cmake/" "C:/src/g2o/cmake_modules")
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
# include Eigen
#enviromental variabels (not tested if strictly neccesary):
#EIGEN3_INCLUDE_DIR=C:\src\eigen
#EIGEN3_DIR=C:\src\eigen\cmake
set(Eigen3_DIR "C:/src/eigen/build_dir")
SET(EIGEN3_INCLUDE_DIR "C:/src/eigen" )
if(NOT EXISTS ${Eigen3_DIR})
    message(FATAL_ERROR "Please run \"git clone https://gitlab.com/libeigen/eigen.git\" or set the directory path above to the directory you ran the command in.")
endif()
find_package(Eigen3 CONFIG REQUIRED)
message(STATUS "Eigen3 FOUND: ${Eigen3_FOUND}")

#include opencv
set(OpenCV_DIR "C:/src/opencv/my_build/install")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OpenCV_FOUND}")
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_dnn)
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# include g20
set(g2o_DIR "C:/src/g2o/lib/cmake/g2o")  # folder with g2oConfig.cmake and g2oTargets.cmake
set(G2O_INCLUDE_DIR "C:/src/g2o/include") # header files (in subdirectories)
set(G2O_ROOT "C:/src/g2o")
set(EIGEN3_ROOT "C:/src/eigen")
set(EIGEN3_ROOT_DIR "C:/src/eigen")
#C:/src/g2o/build/CMakeFiles/Export/6f82a6cf743d92c14565b42826281b56/g2oTargets.cmake
MESSAGE(STATUS "CMAKE_CURRENT_LIST_DIR = ${CMAKE_CURRENT_LIST_DIR}")
#find_package(g2o CONFIG REQUIRED)
message(STATUS "G2O FOUND: ${G2O_FOUND}")
message(STATUS "G2O FOUND stuff: ${G2O_STUFF_LIBRARY}")
message(STATUS "G2O FOUND core: ${G2O_CORE_LIBRARY}")
message(STATUS "G2O INCLUDE_DIR: ${G2O_INCLUDE_DIR}")
message(STATUS "G2O_SOLVERS_FOUND=${G2O_SOLVERS_FOUND}")
set(g2o_LIBS "C:/src/g2o/bin/g2o.exe")  # g2oTarges.cmake.line22
#set(g2o_LIBS C:/src/g2o/lib/libg2o_core.dll.a C:/src/g2o/lib/libg2o_stuff.dll.a C:/src/g2o/lib/libg2o_csparse_extension.dll.a C:/src/g2o/lib/libg2o_types_slam2d.dll.a)
message(STATUS "G2O include dir: ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS}")
if(G2O_FOUND)
    MESSAGE(STATUS "Found g2o")
    set(G2O_BUILD_DIR ${G2O_INCLUDE_DIR}/build)
    set(G2O_ALL_LIBS ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
    add_subdirectory(src)
endif()

include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} C:/src/g2o/bin)  #

add_executable(state_estimator src/main.cpp src/velocity_estimator.cpp src/VisualPipeline.cpp src/VisualPipeline.h src/StateEstimator.cpp src/StateEstimator.h src/SLAMg2o.cpp src/SLAMg2o.h)
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}, g2o_LIBS=${g2o_LIBS}")
target_link_libraries(state_estimator ${OpenCV_LIBS} ${g2o_LIBS})  #
#https://stackoverflow.com/questions/35984678/use-opencv-with-clion-ide-on-windows

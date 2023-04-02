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
set(OpenCV_DIR "C:/src/opencv/my_build/install")  # dir that contains OpenCVConfig.cmake
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV FOUND: ${OpenCV_FOUND}")
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_dnn)
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# include g2o
set(g2o_DIR "C:/src/g2o/lib/cmake/g2o")  # folder with config files
set(g2o_INCLUDE_DIR "C:/src/g2o/include/g2o/core" "C:/src/g2o/include/g2o/solvers" "C:/src/g2o/include/g2o/stuff" "C:/src/g2o/include/g2o/types") # header files (in subdirectories)
# all .dll.a files in g2o/lib: libg2o_cli.dll.a libg2o_core.dll.a libg2o_ext_freeglut_minimal.dll.a libg2o_hierarchical.dll.a libg2o_interface.dll.a libg2o_opengl_helper.dll.a libg2o_parser.dll.a libg2o_simulator.dll.a libg2o_solver_dense.dll.a libg2o_solver_eigen.dll.a libg2o_solver_pcg.dll.a libg2o_solver_slam2d_linear.dll.a libg2o_solver_structure_only.dll.a libg2o_stuff.dll.a libg2o_types_data.dll.a libg2o_types_icp.dll.a libg2o_types_sba.dll.a libg2o_types_sclam2d.dll.a libg2o_types_sim3.dll.a libg2o_types_slam2d.dll.a libg2o_types_slam2d_addons.dll.a libg2o_types_slam3d.dll.a libg2o_types_slam3d_addons.dll.a
#set(g2o_LIBS C:/src/g2o/lib/libg2o_core.dll.a C:/src/g2o/lib/libg2o_stuff.dll.a C:/src/g2o/lib/libg2o_csparse_extension.dll.a C:/src/g2o/lib/libg2o_types_slam2d.dll.a)
#set(g2o_LIBS libg2o_cli.dll.a libg2o_core.dll.a libg2o_ext_freeglut_minimal.dll.a libg2o_hierarchical.dll.a libg2o_interface.dll.a libg2o_opengl_helper.dll.a libg2o_parser.dll.a libg2o_simulator.dll.a libg2o_solver_dense.dll.a libg2o_solver_eigen.dll.a libg2o_solver_pcg.dll.a libg2o_solver_slam2d_linear.dll.a libg2o_solver_structure_only.dll.a libg2o_stuff.dll.a libg2o_types_data.dll.a libg2o_types_icp.dll.a libg2o_types_sba.dll.a libg2o_types_sclam2d.dll.a libg2o_types_sim3.dll.a libg2o_types_slam2d.dll.a libg2o_types_slam2d_addons.dll.a libg2o_types_slam3d.dll.a libg2o_types_slam3d_addons.dll.a)
set(g2o_LIBS g2o::core g2o::stuff g2o::types_slam2d)
find_package(g2o REQUIRED)
message(STATUS "G2O FOUND = ${g2o_FOUND}")
MESSAGE(STATUS "CMAKE_CURRENT_LIST_DIR = ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "g2o_INCLUDE_DIR = ${g2o_INCLUDE_DIR}")
message(STATUS "G2O FOUND stuff = ${g2o_STUFF_LIBRARY}")
message(STATUS "G2O FOUND core = ${g2o_CORE_LIBRARY}")
message(STATUS "G2O_SOLVERS_FOUND = ${g2o_SOLVERS_FOUND}")
message(STATUS "G2O INCLUDE_DIR = ${g2o_INCLUDE_DIR}")


# fin
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${g2o_INCLUDE_DIR})

add_executable(state_estimator src/main.cpp src/velocity_estimator.cpp src/VisualPipeline.cpp src/VisualPipeline.h src/StateEstimator.cpp src/StateEstimator.h src/SLAMg2o.cpp src/SLAMg2o.h src/util.h src/util.cpp src/custom_g2otypes.cpp src/custom_g2otypes.h)
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
message(STATUS "g2o_LIBS = ${g2o_LIBS}")
target_link_libraries(state_estimator ${OpenCV_LIBS} ${g2o_LIBS})
#https://stackoverflow.com/questions/35984678/use-opencv-with-clion-ide-on-windows

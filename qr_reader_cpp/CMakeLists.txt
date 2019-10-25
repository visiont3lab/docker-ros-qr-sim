cmake_minimum_required(VERSION 2.8.3)
project(qr_reader)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)

set(DIR_Lib $ENV{THERMOHUMAN}/lib)
set(DIR_OpenCV ${DIR_Lib}/opencv)


set(OpenCV_DIR ${DIR_OpenCV}/release)
find_package( OpenCV REQUIRED PATHS ${DIR_OpenCV}/cmake)


include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(run_qr_reader qr_reader_pyzbar.cpp )
target_link_libraries( run_qr_reader ${OpenCV_LIBS} zbar )


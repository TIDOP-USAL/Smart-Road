CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
PROJECT(SMART-ROAD)

if(NOT MSVC)
	SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -pedantic -Wno-long-long")
ENDIF()

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)

add_subdirectory(mms_preprocessing)
add_subdirectory(mms_viewer)
######################### External Libraries ###########################

IF(DEFINED ENV{QT5_DIR})
  SET(QT5_DIR $ENV{QT5_DIR})
ENDIF()

SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT5_DIR})


find_package(PCL 1.7 REQUIRED COMPONENTS common filters io octree outofcore)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(Qt5Core)

######################## Executables / Libraries ######################


ADD_EXECUTABLE(mms_preprocessing  mms_preprocessing.cpp)           
target_link_libraries(mms_preprocessing ${PCL_LIBRARIES}
                                ${Boost_LIBRARIES}
                                las
                                Qt5::Core
                                )


INSTALL(TARGETS mms_preprocessing
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY include  DESTINATION include)
######################### External Libraries ###########################

find_package(PCL 1.7 REQUIRED COMPONENTS common filters io octree outofcore)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

######################## Executables / Libraries ######################


ADD_EXECUTABLE(mms_preprocessing  mms_preprocessing.cpp)           
target_link_libraries(mms_preprocessing ${PCL_LIBRARIES}
                                ${Boost_LIBRARIES}
                                las)


INSTALL(TARGETS mms_preprocessing
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY include  DESTINATION include)
######################### External Libraries ###########################

IF(DEFINED ENV{QT5_DIR})
  SET(QT5_DIR $ENV{QT5_DIR})
ENDIF()

SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT5_DIR})

find_package(PCL 1.7 REQUIRED COMPONENTS 
      io octree filters search kdtree  outofcore visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5OpenGL)

FIND_PACKAGE(OpenSceneGraph REQUIRED COMPONENTS osgDB osgGA osgUtil osgViewer)

include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
link_directories(${OPENSCENEGRAPH_LIBRARY_DIRS})
link_directories(/usr/lib/osgPlugins-3.0.1/)

INCLUDE_DIRECTORIES(OSGWidget)
INCLUDE_DIRECTORIES(viewer3D)

SET(WITH_PICK_HANDLER         ON CACHE BOOL "Build with pick handler support")
SET(WITH_SELECTION_PROCESSING ON CACHE BOOL "Build with selection processing support")

######################## Executables / Libraries ######################


add_library(osg_widget  OSGWidget/OSGWidget.cpp
                        OSGWidget/PickHandler.cpp)
target_link_libraries(osg_widget  ${OPENSCENEGRAPH_LIBRARIES}
                                  Qt5::Core
                                  Qt5::Gui
                                  Qt5::OpenGL)

add_library(viewer_3d 	viewer3D/viewer_3d.cpp 
                        viewer3D/pcloud/point_cloud.cpp 
                        viewer3D/pcloud/point_cloud_reader.cpp
                        viewer3D/pcloud/outofcore_octree_reader.cpp)           
target_link_libraries(viewer_3d ${PCL_LIBRARIES}
                                ${OPENSCENEGRAPH_LIBRARIES}
                                ${Boost_LIBRARIES}
                                osgDB
                                osg_widget)

ADD_EXECUTABLE(qtosg  app/MainWindow.cpp
                      app/QtOSG.cpp)

IF(WITH_PICK_HANDLER)
  MESSAGE(STATUS "Building with pick handler support")
  SET_PROPERTY(TARGET osg_widget
              APPEND PROPERTY COMPILE_DEFINITIONS WITH_PICK_HANDLER)
ENDIF()

IF(WITH_SELECTION_PROCESSING)
  MESSAGE(STATUS "Building with selection processing support")
  SET_PROPERTY(TARGET osg_widget
              APPEND PROPERTY COMPILE_DEFINITIONS WITH_SELECTION_PROCESSING)
ENDIF()

TARGET_LINK_LIBRARIES(qtosg osg_widget viewer_3d
                            ${PCL_LIBRARIES}
                            ${OPENSCENEGRAPH_LIBRARIES}
                            Qt5::Core
                            Qt5::Gui
                            Qt5::OpenGL)

INSTALL(TARGETS osg_widget qtosg viewer_3d
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY include  DESTINATION include)

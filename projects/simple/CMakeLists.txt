#project( Project_Simple )

#set( PROJ_SIMPLE_SOURCES SimpleTriangulation.cpp ) # Setup the list of sources here.
find_package(RapidJSON REQUIRED )

include_directories( ./
                     ../
                    "${PROJECT_SOURCE_DIR}/opensource"
                    "${PROJECT_SOURCE_DIR}/common_source"
                    )
#add_library( Project_Simple ${PROJ_SIMPLE_SOURCES} )
add_executable(SimplePointTracking SimplePointTracking.cpp)

# You can refer to other libraries which should be available from the root cmakelists.
link_directories( ${PROJECT_BINARY_DIR}/common_source
                  ${PROJECT_BINARY_DIR}/opensource )
target_link_libraries( SimplePointTracking CommonSource_static
                                           OpenSource_static
                                           ${OpenCV_LIBS}
                                           ${Qt5Core_LIBRARIES}
                                           ${Qt5Widgets_LIBRARIES}
                                           ${Qt5Concurrent_LIBRARIES}
                                           ${Qt5Gui_LIBRARIES}
                                           ${Qt5OpenGL_LIBRARIES}
                                           ${Qt5Sql_LIBRARIES}
                                           ${PCL_LIBRARIES}
                                           ${VTK_LIBRARIES}
                                           )

#${VTK_LIBRARIES}


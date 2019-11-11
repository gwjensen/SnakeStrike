add_subdirectory( Multiview3dRecon-KanataniBook )
add_subdirectory( munkres )

include_directories( ${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/Multiview3dRecon-KanataniBook"  )
include_directories( ${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/munkres" )

add_library( OpenSource_static STATIC distance_3d_seg.cpp )
add_library( OpenSource SHARED distance_3d_seg.cpp )

target_link_libraries( OpenSource_static Multiview3dRecon-KanataniBook_static
                                         Munkres_static
                                          )

target_link_libraries( OpenSource Multiview3dRecon-KanataniBook
                                  Munkres
                                   )

install( TARGETS OpenSource_static OpenSource DESTINATION lib)

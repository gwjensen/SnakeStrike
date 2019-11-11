project( Multiview3dRecon-KanataniBook )

include_directories( "${PROJECT_SOURCE_DIR}/check" "${PROJECT_SOURCE_DIR}/srcs" )

set( MULTIVIEW_SOURCES srcs/multipleview_triangulation.cc 
						srcs/MultipleViewTriangulation_common.cc 
						srcs/optimal_correction.cc 
						srcs/optimal_triplet_correction.cc
						srcs/triplet_triangulation.cc 
						check/fileio.cc ) # Setup the list of sources here.

add_library( Multiview3dRecon-KanataniBook_static STATIC ${MULTIVIEW_SOURCES} )
add_library( Multiview3dRecon-KanataniBook SHARED ${MULTIVIEW_SOURCES} )


# You can refer to other libraries which should be available from the root cmakelists.
#target_link_libraries( Multiview3dRecon-KanataniBook  )
install( TARGETS Multiview3dRecon-KanataniBook_static Multiview3dRecon-KanataniBook DESTINATION lib)

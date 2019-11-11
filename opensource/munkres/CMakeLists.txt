project( Munkres )

include_directories( "${PROJECT_SOURCE_DIR}" )

set( MUNKRES_SOURCES Munkres.cpp ) # Setup the list of sources here.

add_library( Munkres_static STATIC ${MUNKRES_SOURCES} )
add_library( Munkres SHARED ${MUNKRES_SOURCES} )

install( TARGETS Munkres_static Munkres DESTINATION lib)

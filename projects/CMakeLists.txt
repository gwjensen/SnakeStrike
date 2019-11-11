
#include_directories( "${PROJECT_SOURCE_DIR}/simple" "${PROJECT_SOURCE_DIR}/gui" )
add_subdirectory( simple )
add_subdirectory( gui )
add_subdirectory( plugins )#this must be after gui because it is dependent on it





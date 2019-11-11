set( PLUGIN_NAME MatchAlgorithm)

set(CMAKE_CXX_STANDARD 11)

include_directories( .
                   ${OpenCV_INCLUDE_DIRS}
                   "${PROJECT_SOURCE_DIR}/common_source/"
                   "${PROJECT_SOURCE_DIR}/common_source/io"
                   "${PROJECT_SOURCE_DIR}/common_source/image"
                   "${PROJECT_SOURCE_DIR}/common_source/processing" )

####################################################
###:NOTE: I use public functions from common_source/processing/Correspondence.cpp to do basic
###Matching. I'm building with these as their symbols wouldn't be included in gui binary.
add_library( "${PLUGIN_NAME}" SHARED "${PROJECT_SOURCE_DIR}/common_source/MatchingAlgorithmBase.h"
                                     MatchingAlgorithm.h
                                     MatchMaker.h
                                     "${PROJECT_SOURCE_DIR}/common_source/processing/Correspondence.cpp"
                                     MatchingAlgorithm.cpp
                                     MatchMaker.cpp)

link_directories( "${PROJECT_SOURCE_DIR}/common_source" )
#target_link_libraries( "${PLUGIN_NAME}" CommonSource )


install( TARGETS "${PLUGIN_NAME}" DESTINATION lib )
install( FILES  "MatchMaker.h"
                "${PROJECT_SOURCE_DIR}/common_source/MatchingAlgorithmBase.h"
                MatchingAlgorithm.h
         DESTINATION include )



#set(OpenCV_DIR "/usr/local/lib")
#set(Eigen_DIR "/usr/local/include/eigen3")
set( CV_COMPONENTS core
                    highgui
                    imgproc
                    imgcodecs
                    videoio
                    calib3d
                    ccalib
                    features2d
                    xfeatures2d
                    sfm
                    video
                    photo
                    optflow)

find_package(OpenCV REQUIRED COMPONENTS ${CV_COMPONENTS})

find_package(PCL REQUIRED )

include_directories(  "${PROJECT_SOURCE_DIR}"
                        "${PROJECT_SOURCE_DIR}/common_source/image"
                        "${PROJECT_SOURCE_DIR}/common_source/processing"
                        "${PROJECT_SOURCE_DIR}/common_source/io"
                        "${PROJECT_SOURCE_DIR}/common_source/utils"
                        "${PROJECT_SOURCE_DIR}/common_source/visualization")


set( COMMON_SOURCES processing/Correspondence.cpp
                    processing/CamMats.cpp
                    processing/Triangulation.cpp
                    processing/MultiPointTracker.cpp
                    processing/MultiPointSmoother3d.cpp
                    image/ImageSet.cpp
                    image/PixelCluster.cpp
                    image/PixelClusterSet.cpp
                    image/PixelSet.cpp
                    image/SmtImage.cpp
                    image/SmtPixel.cpp
                    io/File.cpp
                    utils/ErrorHandling.cpp
                    utils/utils.cpp
                    visualization/opencv_viz.cpp
                    visualization/pcl_viz.cpp
                    visualization/utils.cpp
                    TriangulationPipeline.cpp) # Setup the list of sources here.

set( COMMON_HEADERS "${PROJECT_SOURCE_DIR}/common_source/image/ImageSet.h"
                    "${PROJECT_SOURCE_DIR}/common_source/image/PixelCluster.h"
                    "${PROJECT_SOURCE_DIR}/common_source/image/PixelClusterSet.h"
                    "${PROJECT_SOURCE_DIR}/common_source/image/PixelSet.h"
                    "${PROJECT_SOURCE_DIR}/common_source/image/SmtImage.h"
                    "${PROJECT_SOURCE_DIR}/common_source/image/SmtPixel.h"
                    "${PROJECT_SOURCE_DIR}/common_source/io/File.h"
                    "${PROJECT_SOURCE_DIR}/common_source/processing/CamMats.h"
                    "${PROJECT_SOURCE_DIR}/common_source/processing/Correspondence.h"
                    "${PROJECT_SOURCE_DIR}/common_source/processing/MultiPointSmoother3d.h"
                    "${PROJECT_SOURCE_DIR}/common_source/processing/MultiPointTracker.h"
                    "${PROJECT_SOURCE_DIR}/common_source/processing/Triangulation.h"
                    "${PROJECT_SOURCE_DIR}/common_source/utils/ErrorHandling.h"
                    "${PROJECT_SOURCE_DIR}/common_source/utils/utils.h"
                    "${PROJECT_SOURCE_DIR}/common_source/visualization/opencv_viz.cpp"
                    "${PROJECT_SOURCE_DIR}/common_source/visualization/pcl_viz.h"
                    "${PROJECT_SOURCE_DIR}/common_source/common.h"
                    "${PROJECT_SOURCE_DIR}/common_source/common_types.h"
                    "${PROJECT_SOURCE_DIR}/common_source/ThreadPool.h"
                    "${PROJECT_SOURCE_DIR}/common_source/TriangulationPipeline.h"
                    "${PROJECT_SOURCE_DIR}/common_source/MatchingAlgorithmBase.h" )

add_library( CommonSource_static STATIC ${COMMON_SOURCES} )
add_library( CommonSource SHARED ${COMMON_SOURCES} )

# You can refer to other libraries which should be available from the root cmakelists.
link_directories( "${PROJECT_SOURCE_DIR}/opensource" )
target_link_libraries( CommonSource_static OpenSource_static ${OpenCV_LIBS} ${PCL_LIBRARIES} )
target_link_libraries( CommonSource OpenSource ${OpenCV_LIBS} ${PCL_LIBRARIES} )

install( TARGETS CommonSource_static CommonSource DESTINATION lib )
install( FILES ${COMMON_HEADERS} DESTINATION include )

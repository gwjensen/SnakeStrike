cmake_minimum_required( VERSION 3.0 )

project(tracking)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic -fno-sized-deallocation -std=c++11 -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-parameter -Wc++11-compat")
set(QMAKE_CXXFLAGS = "${QMAKE_CXXFLAGS} -fno-sized-deallocation")
set(CMAKE_BUILD_TYPE Debug)

find_package(Doxygen
             REQUIRED dot
             OPTIONAL_COMPONENTS mscgen dia)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core
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
                                        optflow)
find_package(OpenMVG REQUIRED COMPONENTS sfm)
find_package(Qt5 COMPONENTS Core Widgets Concurrent Gui OpenGL Sql REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Sql REQUIRED)
find_package(PCL 1.7 REQUIRED )


find_package(VTK 8.0 REQUIRED )
#include(${VTK_USE_FILE}) 


#find_package(Eigen REQUIRED COMPONENTS core )


include_directories(${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    /opt/pylon5/include   )

include_directories ("${PROJECT_SOURCE_DIR}/opensource")
add_subdirectory ( opensource )

include_directories ("${PROJECT_SOURCE_DIR}/common_source")
add_subdirectory ( common_source )

set (BASE_LIBS ${BASE_LIBS} opensource common_source)

include_directories ("${PROJECT_SOURCE_DIR}/projects")
add_subdirectory ( projects )

set(CMAKE_INSTALL_PREFIX /opt/tos)



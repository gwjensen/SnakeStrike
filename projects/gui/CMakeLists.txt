set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) #Runs MOC to generate needed support for form files
set(CMAKE_AUTOUIC ON) #makes cmake look for .ui form files
set(CMAKE_AUTORCC ON) #handles resource files like icons
qt5_add_resources(QT_RESOURCE qtres.qrc)

find_package(RapidJSON REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs videoio calib3d ccalib features2d xfeatures2d sfm video optflow)
find_package(OpenMVG REQUIRED COMPONENTS sfm)
find_package(Qt5 COMPONENTS Core Widgets Concurrent Gui OpenGL Sql REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Sql REQUIRED)
find_package(PCL 1.7 REQUIRED )

find_package( VTK REQUIRED)


if( VTK_FOUND )

include( ${VTK_USE_FILE} )

endif()

link_directories( ${PROJECT_BINARY_DIR}/common_source
                  ${PROJECT_BINARY_DIR}/opensource
                  ${PROJECT_BINARY_DIR}/projects/basler
                  )

include_directories( .
                   ${PROJECT_SOURCE_DIR}/projects/gui
                   ${OpenCV_INCLUDE_DIRS}
                   "/opt/pylon5/include"
                   ${Qt5Core_INCLUDE_DIRS}
                   ${Qt5Widgets_INCLUDE_DIRS}
                   ${Qt5Concurrent_INCLUDE_DIRS}
                   ${Qt5Gui_INCLUDE_DIRS}
                   ${Qt5OpenGL_INCLUDE_DIRS}
                   ${Qt5Sql_INCLUDE_DIRS}
                   ${Qt5Xml_INCLUDE_DIRS}
                   ${PCL_INCLUDE_DIRS}
                   ${VTK_INCLUDE_DIRS}
                   "${PROJECT_SOURCE_DIR}/opensource"
                   "${PROJECT_SOURCE_DIR}/opensource/EPVH/"
                   "${PROJECT_SOURCE_DIR}/common_source"
                  )


set(SOURCES
        icons.qrc
        main.cpp
        MainWindow.cpp
        GuiImage.cpp
        GuiCamera.cpp
        GuiCamInfo.cpp
        GuiCamArray.cpp
        GuiDevice.cpp
        CamPreviewWindow.cpp
        CamPreviewArea.cpp
        CamPreviewCanvas.cpp
        RecordDialog.cpp
        CamCalibrateDialog.cpp
        TrackingDialog.cpp
        CamInfoTree.cpp
        CamTreeItem.cpp
        CountdownDialog.cpp
        ThresholdDialog.cpp
        VisualizationDialog.cpp
        BufferingDialog.cpp
        ExportDialog.cpp
        GrabAndSaveImageTask.cpp
        ProjectDialog.cpp
        TriangulationTask.cpp
        HelpDialog.cpp
        CorrespondenceDialog.cpp
        CorrespondenceImage.cpp

)

set(CAM_SOURCES  GuiImage.cpp
                GuiCamera.cpp
                GuiCamInfo.cpp
                GuiCamArray.cpp
                GuiDevice.cpp
                GrabAndSaveImageTask.cpp


)

set( LIBRARY_LIST ${OpenCV_LIBS}
                    ${Qt5Core_LIBRARIES}
                    ${Qt5Widgets_LIBRARIES}
                    ${Qt5Concurrent_LIBRARIES}
                    ${Qt5Gui_LIBRARIES}
                    ${Qt5OpenGL_LIBRARIES}
                    ${Qt5Sql_LIBRARIES}
                    ${PCL_LIBRARIES}
                   )

add_executable( SnakeStrike ${SOURCES})

target_link_libraries( SnakeStrike  ${LIBRARY_LIST} CommonSource_static OpenSource_static)

add_library( TrackerGui SHARED ${CAM_SOURCES})

target_link_libraries( TrackerGui ${LIBRARY_LIST})

install(TARGETS SnakeStrike DESTINATION bin)

install (FILES "${PROJECT_SOURCE_DIR}/projects/gui/BufferingDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamCalibrateDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/MainWindow.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/GuiImage.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/GuiCamera.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/GuiCamInfo.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/GuiCamArray.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/GuiDevice.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamPreviewWindow.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamPreviewArea.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamPreviewCanvas.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/RecordDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamCalibrateDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/TrackingDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamInfoTree.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CamTreeItem.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/CountdownDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/ThresholdDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/VisualizationDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/BufferingDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/ExportDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/GrabAndSaveImageTask.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/ProjectDialog.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/TriangulationTask.h"
               "${PROJECT_SOURCE_DIR}/projects/gui/HelpDialog.h"
    DESTINATION include)


install( TARGETS TrackerGui DESTINATION lib)



set( PLUGIN_NAME BaslerCams)
#:NOTE:-Wl,--no-undefined is there because if the linking fails on a shared library it fails silently
# We don't want it to fail silently because
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-undefined")

#:TRICKY:If you don't include these flags for autogenerating the QT code, you will get errors about vtables
#missing and you will have a really hard time figuring out why.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) #Runs MOC to generate needed support for form files
set(CMAKE_AUTOUIC ON) #makes cmake look for .ui form files
set(CMAKE_AUTORCC ON) #handles resource files like icons

include_directories( .
                   ${OpenCV_INCLUDE_DIRS}
                   "/opt/pylon5/include"
                   ${Qt5Core_INCLUDE_DIRS}
                   ${Qt5Widgets_INCLUDE_DIRS}
                   ${Qt5Concurrent_INCLUDE_DIRS}
                   ${Qt5Gui_INCLUDE_DIRS}
                   ${Qt5OpenGL_INCLUDE_DIRS}
                   ${Qt5Sql_INCLUDE_DIRS}
                   ${Qt5Xml_INCLUDE_DIRS}
                   "${PROJECT_SOURCE_DIR}/common_source"
                   "${PROJECT_SOURCE_DIR}/projects/gui"
                  )

set( BASLERCAMERA_SOURCES HardwareTriggerConfiguration.h
                          BaslerCamInfo.cpp
                          BaslerDevice.cpp
                          BaslerCamera.cpp
                          CamMaker.cpp
                          DeviceMaker.cpp
                          )

set( BASLERCAMERA_HEADERS BaslerCamInfo.h
                            BaslerDevice.h
                            BaslerCamera.h
                            CamMaker.h
                            DeviceMaker.h)
set(PYLON_ROOT /opt/pylon5)
execute_process(COMMAND ${PYLON_ROOT}/bin/pylon-config --cflags OUTPUT_VARIABLE PYLON_CXX_FLAGS)

if (NOT ${PYLON_CXX_FLAGS})
    string(STRIP ${PYLON_CXX_FLAGS} PYLON_CXX_FLAGS)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PYLON_CXX_FLAGS}")

execute_process(COMMAND shell ${PYLON_ROOT}/bin/pylon-config --libs-rpath OUTPUT_VARIABLE PYLON_LINKER_FLAGS)

if (NOT ${PYLON_LINKER_FLAGS})
    string(STRIP ${PYLON_LINKER_FLAGS} PYLON_LINKER_FLAGS)
endif()

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${PYLON_LINKER_FLAGS}")
set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${PYLON_LINKER_FLAGS}")

add_library( "${PLUGIN_NAME}_static" STATIC ${BASLERCAMERA_HEADERS} ${BASLERCAMERA_SOURCES}  )
add_library( "${PLUGIN_NAME}" SHARED ${BASLERCAMERA_HEADERS} ${BASLERCAMERA_SOURCES} )

set (BASLERCAMERA_LIBRARIES -L/opt/pylon5/lib64
                            -lFirmwareUpdate_gcc_v3_1_Basler_pylon
                            -llog4cpp_gcc_v3_1_Basler_pylon
                            -lLog_gcc_v3_1_Basler_pylon
                            -lMathParser_gcc_v3_1_Basler_pylon
                            -lNodeMapData_gcc_v3_1_Basler_pylon
                            -lXmlParser_gcc_v3_1_Basler_pylon
                            -lGCBase_gcc_v3_1_Basler_pylon
                            -lGenApi_gcc_v3_1_Basler_pylon
                            -lpylonbase
                            -lpylonc
                            -lpylonutility
                            -luxapi
                            -lpylon_TL_usb                            
                            )

execute_process(COMMAND ${PYLON_ROOT}/bin/pylon-config --libs OUTPUT_VARIABLE PYLON_LIB_FLAGS)

if (NOT ${PYLON_LIB_FLAGS})
    string(STRIP ${PYLON_LIB_FLAGS} PYLON_LIB_FLAGS)
endif()

target_link_libraries(  "${PLUGIN_NAME}_static" TrackerGui
                                                CommonSource_static
                                                ${PYLON_LIB_FLAGS}
                                                )
target_link_libraries( "${PLUGIN_NAME}" TrackerGui
                                        CommonSource
                                        ${PYLON_LIB_FLAGS}
                                        )


install( TARGETS "${PLUGIN_NAME}" "${PLUGIN_NAME}_static" DESTINATION lib )
install( FILES  ${BASLERCAMERA_HEADERS} DESTINATION include )

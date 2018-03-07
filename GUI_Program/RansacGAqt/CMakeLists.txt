cmake_minimum_required (VERSION 3.2 FATAL_ERROR)
project(ransac_ga_qt)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/outputDirectory)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libfreenect2pclgrabber-master/modules)


find_package (Qt4 REQUIRED)
#find_package (Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Freenect2 REQUIRED)


set  (project_SOURCES src/main.cpp src/window/pclviewer.cpp src/objects/objects_body.cpp src/ransac/ransac_body.cpp)
set  (project_HEADERS include/pclviewer.h include/objects.h include/ransac.h)
set  (project_FORMS ui/pclviewer.ui)

QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(headers SOURCES ${LibFiles})
include(${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

include_directories (include
            third_party/versor/include
        third_party/versor/include/vsr
        third_party/libfreenect2pclgrabber-master
		${PCL_INCLUDE_DIRS}
		${FREENECT2_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${Qt5_INCLUDE_DIRS}
		${Qt5Widgets_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
)

link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

add_library(versor
        third_party/versor/src/space/vsr_cga3D_op.cpp
        third_party/versor/src/space/vsr_cga3D_frame.cpp
        third_party/versor/src/space/vsr_cga3D_round.cpp
        src/main.cpp
	src/objects/objects_body.cpp
        src/ransac/ransac_body.cpp
        #src/window/pclviewer.cpp
	third_party/libfreenect2pclgrabber-master/k2g.cpp
	)

set(VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)


#qt5_wrap_ui(project_FORMS_HEADERS ${project_FORMS})
#qt5_wrap_cpp(project_HEADERS_MOC   ${project_HEADERS})


ADD_EXECUTABLE  (main ${project_SOURCES} ${QT_SOURCES} ${project_FORMS_HEADERS} ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (main ${QT_LIBRARIES} versor ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${FREENECT2_LIBRARY} ${OpenCV_LIBS} ${VTK_LIBRARIES})





cmake_minimum_required (VERSION 3.2 FATAL_ERROR)
project(example)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

find_package (PCL 1.7.1 REQUIRED)




set  (project_SOURCES main.cpp ../Objects_and_RANSAC_source_code/src/objects_body.cpp ../Objects_and_RANSAC_source_code/src/ransac_body.cpp)
set  (project_HEADERS ../Objects_and_RANSAC_source_code/include/objects.h ../Objects_and_RANSAC_source_code/include/ransac.h)

include_directories (../Objects_and_RANSAC_source_code/include
            ../GUI_Program/RansacGAqt/third_party/versor/include
        ../GUI_Program/RansacGAqt/third_party/versor/include/vsr
		${PCL_INCLUDE_DIRS}
)

link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

add_library(versor
        ../GUI_Program/RansacGAqt/third_party/versor/src/space/vsr_cga3D_op.cpp
        ../GUI_Program/RansacGAqt/third_party/versor/src/space/vsr_cga3D_frame.cpp
        ../GUI_Program/RansacGAqt/third_party/versor/src/space/vsr_cga3D_round.cpp
        main.cpp
	../Objects_and_RANSAC_source_code/src/objects_body.cpp
        ../Objects_and_RANSAC_source_code/src/ransac_body.cpp
	)


ADD_EXECUTABLE  (main ${project_SOURCES} ${project_HEADERS})

TARGET_LINK_LIBRARIES (main versor ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


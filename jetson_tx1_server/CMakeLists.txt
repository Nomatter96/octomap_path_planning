cmake_minimum_required (VERSION 3.5.1)
project (jetson_tx1_server)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

set(VTK_DIR "/usr/lib/cmake/vtk-6.2")
find_package(VTK CONFIG REQUIRED)
include(${VTK_USE_FILE})

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(PCL CONFIG REQUIRED COMPONENTS)
find_package(OCTOMAP REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${fcl_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${ZED_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFENITIONS})


set(SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/pcl_server/pcl_server.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/pcl_server/pcl_server.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pcl_server/planner.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pcl_server/planner.cpp"
	"main.cpp" 	
)

add_executable(main ${SOURCES})
add_definitions(-std=c++11 -g -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

target_link_libraries(main PUBLIC ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${ZED_LIBS} ${OpenCV_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} fcl ${OMPL_LIBRARIES})

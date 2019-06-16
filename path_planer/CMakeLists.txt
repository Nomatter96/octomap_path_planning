cmake_minimum_required(VERSION 3.10)

#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(auv_path_panner)

set(SOURCES 
	main.cpp
	application.cpp
	application.hpp
	zed_camera.cpp
	zed_camera.hpp
	octomap_wrapper.cpp
	octomap_wrapper.hpp
	path_planer.cpp
	path_planer.hpp
	)

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(fcl CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(ompl CONFIG REQUIRED)
find_package(octomap CONFIG REQUIRED)
find_package(ccd CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${fcl_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})


add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} fcl ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} ${PCL_LIBRARIES} ${OMPL_LIBRARIES} ccd ${OCTOMAP_LIBRARIES})
 

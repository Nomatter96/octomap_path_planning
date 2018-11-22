# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.11)
set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake")
project (client_server_win)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost COMPONENTS filesystem system thread date_time REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(SOURCES "client_server_win/client_server_win.cpp"
	"client_server_win/client_server_win.h"
	"main.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})

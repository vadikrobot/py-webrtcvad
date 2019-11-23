cmake_minimum_required(VERSION 3.3)
project(example CXX)

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################
# Find thread
################################
find_package(Threads REQUIRED)

##################################
# WebRtc
##################################
set(webrtc_SOURCE_DIR cbits)
add_subdirectory(${webrtc_SOURCE_DIR})

add_executable(example example.cpp)
target_link_libraries(example webrtc)
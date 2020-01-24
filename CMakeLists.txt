cmake_minimum_required(VERSION 3.10)
project(TurtlebotRL)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find_package(SFML COMPONENTS graphics audio REQUIRED)
# find_package(Box2D REQUIRED)
add_subdirectory(simulator)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE simulator)
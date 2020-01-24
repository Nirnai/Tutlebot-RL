cmake_minimum_required(VERSION 3.10)
project(Simulator)


find_package(SFML COMPONENTS graphics REQUIRED)
find_package(Box2D REQUIRED)

add_library(simulator STATIC src/Environment.cpp)

target_include_directories(simulator PUBLIC include)
target_link_libraries(simulator PUBLIC sfml-graphics Box2D)
target_compile_features(simulator PRIVATE cxx_std_14)

cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(newton_sitl_tests)

find_package(MAVSDK REQUIRED)

add_executable(test_takeoff_land test_takeoff_land.cpp)
target_link_libraries(test_takeoff_land MAVSDK::mavsdk)

cmake_minimum_required(VERSION 3.10)
project(ALMG)

find_package(Matlab REQUIRED)
find_package(Boost REQUIRED)

include_directories(${Matlab_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Matlab_LIBRARY_DIRS})
add_definitions(${Matlab_DEFINITIONS})

add_subdirectory(Lanelet2/lanelet2_core)
add_subdirectory(Lanelet2/lanelet2_io)
add_subdirectory(Lanelet2/lanelet2_projection)
add_subdirectory(Lanelet2/lanelet2_routing)
add_subdirectory(Lanelet2/lanelet2_traffic_rules)
add_subdirectory(external/pybind11)

set(SOURCES 
    src/Landmark.cpp 
    src/TripAnalyzer.cpp
)
# file(GLOB_RECURSE SOURCES "src/*cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

pybind11_add_module(ALMG ${SOURCES})
include_directories(ALMG PUBLIC ${CMAKE_SOURCE_DIR}/include)

set_property(TARGET ALMG PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ALMG PUBLIC
                      lanelet2_core
                      lanelet2_io 
                      lanelet2_projection
                      lanelet2_routing
                      lanelet2_traffic_rules
                      ${Matlab_LIBRARIES}
                      ${Boost_LIBRARIES})
set_target_properties(ALMG PROPERTIES
                      CXX_STANDARD 17
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python)

# install(TARGETS research DESTINATION ${CMAKE_SOURCE_DIR}/python)
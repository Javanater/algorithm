cmake_minimum_required(VERSION 3.5)

project(algorithm)

set(SOURCE_FILES
    src/GridAStar.cpp
    src/DifferentialDriveAStar.cpp
    src/DifferentialDriveAStarCont.cpp
    
    include/algorithm/AStar.hpp
    include/algorithm/GridAStar.hpp
    include/algorithm/DStarLite.hpp
    include/algorithm/LPAStar.hpp
    include/algorithm/DifferentialDriveAStar.hpp
    include/algorithm/DifferentialDriveAStarCont.hpp
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_subdirectory(../simulation simulation)
add_subdirectory(../math math)

target_link_libraries(${PROJECT_NAME} PUBLIC Simulation Math)

add_executable(${PROJECT_NAME}_Test test/AStarTest.cpp)

find_package(Boost COMPONENTS unit_test_framework REQUIRED)
find_package(GTest REQUIRED)

target_link_libraries(${PROJECT_NAME}_Test
    ${PROJECT_NAME}
	Boost::unit_test_framework
    GTest::Main
)


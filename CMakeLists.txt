#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild
#  > cmake --build build
#

cmake_minimum_required(VERSION 3.10)

project(patrolling-solver)

include(FetchContent)

# Fetch yaml-cpp
FetchContent_Declare(
	yaml-cpp
	GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
	GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
	message(STATUS "Fetching yaml-cpp...")
	FetchContent_Populate(yaml-cpp)
	add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

# Fetch nlohmann_json
include(FetchContent)
FetchContent_Declare(
	nlohmann_json
	GIT_REPOSITORY https://github.com/nlohmann/json.git
	GIT_TAG v3.10.5
)
FetchContent_MakeAvailable(nlohmann_json)

option(CXX "enable C++ compilation" ON)
if(CXX)
	enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_FLAGS "-O3 -ggdb -g -Wall -Werror")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
endif()

include_directories(
	${CMAKE_CURRENT_LIST_DIR}/inc
)

# list source files here
set(sources src/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}
	${sources}
	src/Input.cpp
	src/KMeansSolver.cpp
	src/PatrollingInput.cpp
	src/Roots.cpp
	src/Solution.cpp
	src/Solver.cpp
	src/Solver_Baseline.cpp
	src/Utilities.cpp
	src/VRPSolver.cpp
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	include(FeatureSummary)
	feature_summary(WHAT ALL)
endif()

# Link yaml-cpp library
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp)
# Link nlohmann_json
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)



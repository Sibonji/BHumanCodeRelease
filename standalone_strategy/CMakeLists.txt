cmake_minimum_required(VERSION 3.16)
project(bhuman_standalone_strategy CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    include
    include/shims
    include/Streaming
    include/Math
    include/MathBase
    include/Strategy
    include/Strategy/ActiveRoles
    include/Strategy/PositionRoles
    include/Tools/BehaviorControl
    include/Tools/Modeling
    include/Tools/Motion
    include/Representations
    include/Representations/BehaviorControl
    include/Representations/Configuration
    include/Representations/Infrastructure
    include/Representations/Communication
    include/Representations/Modeling
    include/Representations/MotionControl
    include/Representations/Sensing
    include/Platform
    include/Debugging
)

add_compile_options(-Wall -Wextra -O2 -Wno-invalid-constexpr -Wno-builtin-declaration-mismatch)

file(GLOB STRATEGY_SOURCES src/*.cpp)

add_library(bhuman_strategy STATIC ${STRATEGY_SOURCES})

add_executable(strategy_test main_test.cpp)
target_link_libraries(strategy_test PRIVATE bhuman_strategy)

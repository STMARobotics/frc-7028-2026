cmake_minimum_required(VERSION 3.25)

project(ballisticSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(ballisticSimulator
        SOTM/src/main.cpp
        SOTM/src/main.h
        SOTM/src/Simulator.cpp
        SOTM/src/Simulator.h
        SOTM/src/BallisticEnvironmentProfile.h
        SOTM/src/BallisticProjectileState.h
        SOTM/src/BallisticSimulatorResolutionProfile.h
)

target_link_libraries(ballisticSimulator PRIVATE Eigen3::Eigen)

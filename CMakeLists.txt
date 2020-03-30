#
# Set minimum required version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(diffusion_example)

#
# add path where cmake files must be searched for
#
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Include path points to the base source dir
#
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})


#
# Add all the CMake modules we need
#
# Set build options
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)
# Set compiler flags
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)
# Check for libraries
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)
# Set FindArgosPlugin
include(${CMAKE_SOURCE_DIR}/cmake/FindARGoSPlugin.cmake)

#
# Check if KheperaIV is installed
#
FindARGoSPlugin(KheperaIV khepera khiv opengl_kheperaiv kheperaiv_entity)

if(NOT ARGOS_KheperaIV_FOUND)
    message(FATAL_ERROR "Khepera plugin not found !!" )    
endif()

add_library(diffusion MODULE src/diffusion.h src/diffusion.cpp)
target_link_libraries(diffusion
  argos3core_simulator
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_genericrobot)

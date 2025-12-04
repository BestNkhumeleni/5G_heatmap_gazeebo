cmake_minimum_required(VERSION 3.10)
project(ign_gnb_heatmap)


# Use ignition-cmake helpers if available
find_package(ignition-cmake2 QUIET)
if(ignition-cmake2_FOUND)
ign_find_package(ignition-rendering6 REQUIRED)
ign_find_package(ignition-transport11 REQUIRED)
else()
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
endif()


add_library(HeatmapRenderPlugin SHARED
src/HeatmapRenderPlugin.cc)


target_include_directories(HeatmapRenderPlugin PRIVATE include)


target_link_libraries(HeatmapRenderPlugin
ignition-rendering6::ignition-rendering6
ignition-transport11::ignition-transport11)


# Install plugin into a plugins folder so ign gazebo can find it
install(TARGETS HeatmapRenderPlugin
LIBRARY DESTINATION lib)
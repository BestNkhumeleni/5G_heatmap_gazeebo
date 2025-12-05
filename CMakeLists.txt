cmake_minimum_required(VERSION 3.10)
project(ign_gnb_heatmap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Ignition packages
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-common4 REQUIRED)

# Create the plugin shared library
add_library(HeatmapPlugin SHARED
    src/HeatmapPlugin.cc
    src/PropagationModels.cc
)

target_include_directories(HeatmapPlugin PRIVATE
    include
    ${IGNITION-GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(HeatmapPlugin
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1-register
    ignition-transport11::ignition-transport11
    ignition-msgs8::ignition-msgs8
    ignition-math6::ignition-math6
    ignition-common4::ignition-common4
)

# Install the plugin
install(TARGETS HeatmapPlugin
    LIBRARY DESTINATION lib
)

# Install world files
install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
)

# Install headers (optional, for development)
install(DIRECTORY include/
    DESTINATION include
)
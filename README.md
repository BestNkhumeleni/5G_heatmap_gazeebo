# GNB Heatmap Render Plugin (Ignition Gazebo 6)


## What this repo contains
A starter rendering plugin for Ignition Gazebo 6 that computes a 2D ground-plane heatmap of received power (dBm) from a configured gNB position using FSPL and renderer ray queries for occlusion. The plugin:


- Runs as a rendering plugin (client-side) and uses a worker thread for CPU work.
- Publishes a `gz::msgs::Image` on `/gnb/heatmap` as an 8-bit RGBA image mapping dBm -> color.
- Includes an example `worlds/heatmap_world.sdf` showing how to load the plugin and configure SDF params.


## Build


```bash
mkdir build && cd build
cmake ..
make -j

## how to run
ign gazebo worlds/heatmap_world.sdf --gui-config gui.config

# optionally: sudo make install (or copy .so to ~/.ignition/gazebo/plugins or a GAZEBO_PLUGIN_PATH)


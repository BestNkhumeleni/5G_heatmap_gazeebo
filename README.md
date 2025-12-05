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

#How to change the propagation model being using during runtime

# Change to ray tracing model
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "ray_tracing"'

# Change to 3GPP UMi
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_umi"'

# Update configuration
ign topic -t /gnb/heatmap/config -m ignition.msgs.StringMsg -p 'data: "tx_power=40;wall_loss=20"'

# Check current status
ign topic -e -t /gnb/heatmap/status -n 1
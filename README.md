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
make

## how to run
ign gazebo worlds/heatmap_world.sdf --gui-config gui.config

#How to change the propagation model being using during runtime

# Change to ray tracing model
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "ray_tracing"'

# Change to 3GPP UMi
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_umi"'

# Change to 3GPP UMa
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_uma"'

# Update configuration
ign topic -t /gnb/heatmap/config -m ignition.msgs.StringMsg -p 'data: "tx_power=40;wall_loss=20"'

##Or use the scipt (run from inside the scripts directory):
chmod +x heatmap_control.sh
./heatmap_control.sh model ray_tracing
./heatmap_control.sh power 40
./heatmap_control.sh status

# Move gNB to new position
./heatmap_control.sh move 5 10 12

# Query signal at a point
./heatmap_control.sh query 15 20 1.5

# Listen for click events
./heatmap_control.sh listen_clicks

# Check current status
ign topic -e -t /gnb/heatmap/status -n 1

# Zoom in 3 steps
./heatmap_control.sh zoom_in 3

# Pan 20 meters to the right
./heatmap_control.sh pan 20 0

# Jump to position (100, 50) at 4x zoom
./heatmap_control.sh goto 100 50 4

# Center on gNB and zoom in
./heatmap_control.sh center
./heatmap_control.sh zoom_in 4

# Interactive keyboard mode
./heatmap_control.sh interactive
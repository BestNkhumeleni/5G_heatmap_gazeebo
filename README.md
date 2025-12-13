# GNB Heatmap Render Plugin (Ignition Gazebo 6)


## What this repo contains
A starter rendering plugin for Ignition Gazebo 6 that computes a 2D ground-plane heatmap of received power (dBm) from a configured gNB position using FSPL and renderer ray queries for occlusion. The plugin:


- Runs as a rendering plugin (client-side) and uses a worker thread for CPU work.
- Publishes a `gz::msgs::Image` on `/gnb/heatmap` as an 8-bit RGBA image mapping dBm -> color.
- Includes an example `worlds/heatmap_world.sdf` showing how to load the plugin and configure SDF params.

## File directory structure:
ign_gnb_heatmap/
├── CMakeLists.txt
├── include/
│   └── heatmap_plugin/
│       ├── HeatmapPlugin.hh
│       └── PropagationModels.hh
├── src/
│   ├── HeatmapPlugin.cc
│   └── PropagationModels.cc
├── worlds/
│   └── heatmap_world.sdf
└── scripts/
    └── heatmap_control.sh


## Build

```bash
mkdir build && cd build
cmake ..
make
```

## how to run

```bash
#From the working directory export the plug in path to gazebo:

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$(pwd)/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH

# and then run this
ign gazebo worlds/word.sdf --gui-config gui.config
```

# Change to ray tracing model
```bash
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "ray_tracing"'
```

# Change to 3GPP UMi
```bash
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_umi"'
```

# Change to 3GPP UMa
```bash
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_uma"'
```

# Update configuration
```bash
ign topic -t /gnb/heatmap/config -m ignition.msgs.StringMsg -p 'data: "tx_power=40;wall_loss=20"'
```

## Using Heatmap control script:
```bash
chmod +x heatmap_control.sh
./heatmap_control.sh model ray_tracing
./heatmap_control.sh power 40
./heatmap_control.sh status
```

# Move gNB to new position
```bash
./heatmap_control.sh move 5 10 12
```

# Query signal at a point
```bash
./heatmap_control.sh query 15 20 1.5
```

# Listen for click events
```bash
./heatmap_control.sh listen_clicks
```

# Check current status
```bash
ign topic -e -t /gnb/heatmap/status -n 1
```

# Zoom in 3 steps
```bash
./heatmap_control.sh zoom_in 3
```

# Zoom out 3 steps
```bash
./heatmap_control.sh zoom_out 3
```

# Pan 20 meters to the right
```bash
./heatmap_control.sh pan 20 0
```

# Jump to position (100, 50) at 4x zoom
```bash
./heatmap_control.sh goto 100 50 4
```

# Center on gNB and zoom in
```bash
./heatmap_control.sh center
./heatmap_control.sh zoom_in 4
```

# Interactive keyboard mode
```bash
./heatmap_control.sh interactive
```

## Running tests on scenarios

```bash
# Copy scenario files to worlds directory

# Copy scenario files to worlds directory
cp scenario_*.sdf worlds/

# Make scripts executable
chmod +x run_comparison.sh

# Run full comparison (headless)
./run_comparison.sh all

# Run single scenario for testing
./run_comparison.sh scenario street_canyon

# Generate analysis figures (after data collection)
pip install pandas matplotlib numpy
python analyze_results.py comparison_results/<timestamp>
```

## Output Structure
```
comparison_results/
└── 20241213_HHMMSS/
    ├── raw_data/
    │   ├── scenario_open_field.csv
    │   ├── scenario_open_field_stats.csv
    │   └── ...
    ├── figures/
    │   ├── signal_vs_distance.pdf
    │   ├── model_comparison_heatmap.pdf
    │   └── coverage_distribution.pdf
    ├── comparison_report.txt
    ├── summary_all_scenarios.csv
    └── latex_tables.tex
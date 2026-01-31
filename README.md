````markdown
# GNB Heatmap Render Plugin (Ignition Gazebo 6)

A rendering plugin for **Ignition Gazebo 6** that computes a **2D ground-plane heatmap** of received power (dBm) from one or more configured gNB positions. It supports **FSPL-based propagation**, **ray-query occlusion**, and multiple propagation models.

---

## Features

- Runs as a **client-side rendering plugin** (uses a worker thread for CPU work).
- Publishes a `gz::msgs::Image` on: `/gnb/heatmap`
  - 8-bit **RGBA** heatmap image (dBm → color mapping).
- Includes an example world file showing plugin loading + SDF configuration:
  - `worlds/heatmap_world.sdf`

---

## Repository Structure

```text
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
````

---

## Build

From the repository root:

```bash
mkdir -p build && cd build
cmake ..
make
```

---

## Run (Gazebo)

### 1) Export plugin path

From the repo root (or adjust paths accordingly):

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$(pwd)/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

### 2) Launch world

> Note: your original README referenced `worlds/word.sdf`, which looks like a typo.
> The example world in this repo is `worlds/heatmap_world.sdf`.

```bash
ign gazebo worlds/heatmap_world.sdf --gui-config gui.config
```

---

## Control the Heatmap (Ignition Topics)

### Switch propagation model

**Ray tracing**

```bash
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "ray_tracing"'
```

**3GPP UMi**

```bash
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_umi"'
```

**3GPP UMa**

```bash
ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p 'data: "3gpp_uma"'
```

### Update configuration (example)

```bash
ign topic -t /gnb/heatmap/config -m ignition.msgs.StringMsg -p 'data: "tx_power=40;wall_loss=20"'
```

### Check plugin status

```bash
ign topic -e -t /gnb/heatmap/status -n 1
```

---

## Using the Control Script (`heatmap_control.sh`)

### Make executable

```bash
chmod +x scripts/heatmap_control.sh
```

> If you run it from the repo root, call it as `./scripts/heatmap_control.sh ...`

### Basic examples

```bash
./scripts/heatmap_control.sh model ray_tracing
./scripts/heatmap_control.sh power 40
./scripts/heatmap_control.sh status
```

---

## gNB Operations

### Move an existing gNB

```bash
./scripts/heatmap_control.sh move 0 5 5 12
```

*(Example: move gNB with ID `0` to x=5, y=5, z=12)*

### Add a new gNB

```bash
./scripts/heatmap_control.sh add 20 15 10 "NewSite"
```

### Update gNB parameters

```bash
./scripts/heatmap_control.sh update 1 tx_power=35;tx_gain=12
```

### List all gNBs

```bash
./scripts/heatmap_control.sh list
```

---

## Query & Interaction

### Query signal at a point

```bash
./scripts/heatmap_control.sh query 15 20 1.5
```

### Listen for click events

```bash
./scripts/heatmap_control.sh listen_clicks
```

### Interactive keyboard mode

```bash
./scripts/heatmap_control.sh interactive
```

---

## View Controls (Zoom / Pan / Navigate)

### Zoom

```bash
./scripts/heatmap_control.sh zoom_in 3
./scripts/heatmap_control.sh zoom_out 3
```

### Pan (example: 20 meters to the right)

```bash
./scripts/heatmap_control.sh pan 20 0
```

### Jump to position (x=100, y=50) at 4× zoom

```bash
./scripts/heatmap_control.sh goto 100 50 4
```

### Center on gNB and zoom in

```bash
./scripts/heatmap_control.sh center
./scripts/heatmap_control.sh zoom_in 4
```

---

## Combined Views (e.g., SINR)

### Switch to SINR view

```bash
./scripts/heatmap_control.sh combine sinr
```

---

## Scenario Testing (Single-gNB Comparisons)

### 1) Copy scenario files

```bash
cp scenario_*.sdf worlds/
```

### 2) Run comparisons

```bash
chmod +x run_comparison.sh

# Run full comparison (headless)
./run_comparison.sh all

# Run single scenario for testing
./run_comparison.sh scenario street_canyon
```

### 3) Analyze results

```bash
pip install pandas matplotlib numpy
python analyze_results.py comparison_results/<timestamp>
```

---

## Output Structure

```text
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
```

---

## Multi-gNB Testing

### 1) Setup

```bash
chmod +x run_multi_gnb_comparison.sh quick_multi_gnb_test.sh
cp scenario_multi_gnb_experiment.sdf worlds/
```

### 2) Run experiments

```bash
# Run full comparison (headless)
./run_multi_gnb_comparison.sh all scenario_multi_gnb_experiment

# Or run specific experiment types
./run_multi_gnb_comparison.sh interference scenario_open_field
./run_multi_gnb_comparison.sh complementarity scenario_dense_urban
```

### 3) Analyze results

```bash
python3 analyze_multi_gnb.py multi_gnb_results/<timestamp>
```

### 4) Interactive testing (with Gazebo running)

```bash
./quick_multi_gnb_test.sh interactive
```

```
```

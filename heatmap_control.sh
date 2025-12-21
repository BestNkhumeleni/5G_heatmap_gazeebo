#!/bin/bash
# Heatmap Plugin Control Script - Multi-gNB Edition
# Usage: ./heatmap_control.sh <command> [args]

# Topics
TOPIC_SET_MODEL="/gnb/heatmap/set_model"
TOPIC_CONFIG="/gnb/heatmap/config"
TOPIC_STATUS="/gnb/heatmap/status"
TOPIC_SET_POSITION="/gnb/heatmap/set_position"
TOPIC_QUERY_POSITION="/gnb/heatmap/query_position"
TOPIC_CLICK_INFO="/gnb/heatmap/click_info"
TOPIC_QUERY_RESULT="/gnb/heatmap/query_result"
TOPIC_ZOOM="/gnb/heatmap/zoom"
TOPIC_PAN="/gnb/heatmap/pan"
TOPIC_SET_VIEW="/gnb/heatmap/set_view"
TOPIC_RESET_VIEW="/gnb/heatmap/reset_view"
TOPIC_CENTER_GNB="/gnb/heatmap/center_on_gnb"
TOPIC_VIEW_INFO="/gnb/heatmap/view_info"

# Multi-gNB topics
TOPIC_ADD_GNB="/gnb/heatmap/add_gnb"
TOPIC_REMOVE_GNB="/gnb/heatmap/remove_gnb"
TOPIC_MOVE_GNB="/gnb/heatmap/move_gnb"
TOPIC_UPDATE_GNB="/gnb/heatmap/update_gnb"
TOPIC_ENABLE_GNB="/gnb/heatmap/enable_gnb"
TOPIC_DISABLE_GNB="/gnb/heatmap/disable_gnb"
TOPIC_LIST_GNBS="/gnb/heatmap/list_gnbs"
TOPIC_GNB_LIST="/gnb/heatmap/gnb_list"
TOPIC_COMBINE_MODE="/gnb/heatmap/set_combine_mode"

show_help() {
    echo "RF Heatmap Plugin Control - Multi-gNB Edition"
    echo ""
    echo "Usage: $0 <command> [arguments]"
    echo ""
    echo "=== Multi-gNB Commands ==="
    echo "  add <x> <y> <z> [name]   - Add new gNB at position"
    echo "  remove <id>              - Remove gNB by ID"
    echo "  move <id> <x> <y> <z>    - Move gNB to new position"
    echo "  enable <id>              - Enable gNB"
    echo "  disable <id>             - Disable gNB"
    echo "  update <id> <params>     - Update gNB parameters"
    echo "                             e.g., update 0 tx_power=35;tx_gain=12"
    echo "  list                     - List all gNBs"
    echo "  watch_list               - Continuously watch gNB list"
    echo ""
    echo "=== Signal Combination ==="
    echo "  combine <mode>           - Set signal combination mode"
    echo "                             Modes: best_server, sum_power, sinr"
    echo ""
    echo "=== View Control Commands ==="
    echo "  zoom_in [amount]         - Zoom in (default: 2 steps)"
    echo "  zoom_out [amount]        - Zoom out (default: 2 steps)"
    echo "  zoom <level>             - Set zoom to specific level"
    echo "  pan <dx> <dy>            - Pan view by dx, dy in world units"
    echo "  goto <x> <y> [zoom]      - Go to position with optional zoom"
    echo "  center [id]              - Center view on gNB (default: 0)"
    echo "  reset                    - Reset view to default"
    echo "  view                     - Show current view info"
    echo ""
    echo "=== Model Commands ==="
    echo "  model <name>             - Switch propagation model"
    echo "                             Options: free_space, 3gpp_umi, 3gpp_uma,"
    echo "                                      ray_tracing, hybrid"
    echo ""
    echo "=== Global Config Commands ==="
    echo "  config <params>          - Update global config"
    echo "  power <dbm>              - Set default transmit power"
    echo "  freq <hz>                - Set default frequency"
    echo "  wall_loss <db>           - Set wall penetration loss"
    echo "  shadowing <on|off>       - Enable/disable shadowing"
    echo ""
    echo "=== Interactive Commands ==="
    echo "  query <x> <y> [z]        - Query signal strength at position"
    echo "  listen_clicks            - Listen to click query results"
    echo ""
    echo "=== Status Commands ==="
    echo "  status                   - Show current status"
    echo "  watch_status             - Continuously watch status"
    echo ""
    echo "=== Demo Commands ==="
    echo "  demo_multi               - Set up demo with 3 gNBs"
    echo "  demo_interference        - Demo interference analysis"
    echo ""
    echo "Examples:"
    echo "  $0 add 20 15 10 'gNB_East'   # Add gNB at (20,15,10) named gNB_East"
    echo "  $0 move 1 30 20 10           # Move gNB 1 to new position"
    echo "  $0 update 0 tx_power=40      # Set gNB 0 power to 40 dBm"
    echo "  $0 combine sinr              # Switch to SINR view"
    echo "  $0 center 2                  # Center view on gNB 2"
}

# ============================================================================
# Multi-gNB Functions
# ============================================================================

add_gnb() {
    local x=$1
    local y=$2
    local z=$3
    local name=${4:-""}
    
    echo "Adding gNB at ($x, $y, $z)${name:+ named '$name'}..."
    ign topic -t $TOPIC_ADD_GNB -m ignition.msgs.Pose \
        -p "name: '$name', position: {x: $x, y: $y, z: $z}"
}

remove_gnb() {
    local id=$1
    echo "Removing gNB [$id]..."
    ign topic -t $TOPIC_REMOVE_GNB -m ignition.msgs.Int32 -p "data: $id"
}

move_gnb() {
    local id=$1
    local x=$2
    local y=$3
    local z=$4
    echo "Moving gNB [$id] to ($x, $y, $z)..."
    ign topic -t $TOPIC_MOVE_GNB -m ignition.msgs.Pose \
        -p "name: '$id', position: {x: $x, y: $y, z: $z}"
}

enable_gnb() {
    local id=$1
    echo "Enabling gNB [$id]..."
    ign topic -t $TOPIC_ENABLE_GNB -m ignition.msgs.Int32 -p "data: $id"
}

disable_gnb() {
    local id=$1
    echo "Disabling gNB [$id]..."
    ign topic -t $TOPIC_DISABLE_GNB -m ignition.msgs.Int32 -p "data: $id"
}

update_gnb() {
    local id=$1
    local params=$2
    echo "Updating gNB [$id]: $params"
    ign topic -t $TOPIC_UPDATE_GNB -m ignition.msgs.StringMsg \
        -p "data: \"id=$id;$params\""
}

list_gnbs() {
    echo "Requesting gNB list..."
    
    # Start listener in background FIRST, then send request
    # This ensures we catch the response
    (
        # Timeout after 2 seconds if no response
        timeout 2 ign topic -e -t $TOPIC_GNB_LIST -n 1 2>/dev/null | grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
            if [ ! -z "$line" ]; then
                echo "  $line"
            fi
        done
    ) &
    LISTENER_PID=$!
    
    # Small delay to ensure listener is ready
    sleep 0.15
    
    # Send the request
    ign topic -t $TOPIC_LIST_GNBS -m ignition.msgs.Empty -p ""
    
    # Wait for listener to complete
    wait $LISTENER_PID 2>/dev/null
    
    echo ""
}

watch_gnb_list() {
    echo "Watching gNB list (Ctrl+C to stop)..."
    ign topic -e -t $TOPIC_GNB_LIST | while read -r line; do
        info=$(echo "$line" | grep -oP 'data: "\K[^"]+')
        if [ ! -z "$info" ]; then
            clear
            echo "=== gNB List [$(date '+%H:%M:%S')] ==="
            echo ""
            echo "$info" | tr ';' '\n' | while read item; do
                if [ ! -z "$item" ]; then
                    echo "  $item"
                fi
            done
        fi
    done
}

set_combine_mode() {
    local mode=$1
    echo "Setting signal combination mode to: $mode"
    ign topic -t $TOPIC_COMBINE_MODE -m ignition.msgs.StringMsg -p "data: \"$mode\""
}

# ============================================================================
# Demo Functions
# ============================================================================

demo_multi() {
    echo "Setting up multi-gNB demo..."
    echo ""
    
    # Add three gNBs in a triangle pattern
    echo "Adding gNB at (0, 0, 10) - Central..."
    add_gnb 0 0 10 "Central"
    sleep 0.5
    
    echo "Adding gNB at (25, 20, 12) - East..."
    add_gnb 25 20 12 "East"
    sleep 0.5
    
    echo "Adding gNB at (-20, 25, 10) - West..."
    add_gnb -20 25 10 "West"
    sleep 0.5
    
    echo ""
    echo "Demo setup complete! Try:"
    echo "  $0 list                    # See all gNBs"
    echo "  $0 combine best_server     # View best server"
    echo "  $0 combine sinr            # View SINR"
    echo "  $0 query 10 10             # Query point between gNBs"
    echo "  $0 disable 1               # Disable East gNB"
}

demo_interference() {
    echo "Setting up interference analysis demo..."
    echo ""
    
    # Add gNBs close together to show interference
    echo "Adding co-located gNBs to demonstrate interference..."
    add_gnb 0 0 10 "gNB_A"
    sleep 0.3
    add_gnb 15 0 10 "gNB_B"
    sleep 0.3
    add_gnb 7.5 13 10 "gNB_C"
    sleep 0.3
    
    echo ""
    echo "Switching to SINR view..."
    set_combine_mode "sinr"
    
    echo ""
    echo "Interference demo setup complete!"
    echo "The heatmap now shows Signal-to-Interference-plus-Noise Ratio."
    echo "Areas with low SINR (blue) indicate high interference."
    echo ""
    echo "Try:"
    echo "  $0 combine best_server    # Switch to coverage view"
    echo "  $0 combine sum_power      # Switch to total power view"
    echo "  $0 update 1 tx_power=40   # Increase gNB_B power"
}

# ============================================================================
# View Control Functions
# ============================================================================

zoom_in() {
    local amount=${1:-2}
    echo "Zooming in ($amount steps)..."
    ign topic -t $TOPIC_ZOOM -m ignition.msgs.Double -p "data: $amount"
}

zoom_out() {
    local amount=${1:-2}
    echo "Zooming out ($amount steps)..."
    ign topic -t $TOPIC_ZOOM -m ignition.msgs.Double -p "data: -$amount"
}

set_zoom() {
    local level=$1
    echo "Setting zoom to ${level}x..."
    ign topic -t $TOPIC_SET_VIEW -m ignition.msgs.Pose \
        -p "position: {x: 0, y: 0, z: $level}"
}

pan_view() {
    local dx=$1
    local dy=$2
    echo "Panning view by ($dx, $dy)..."
    ign topic -t $TOPIC_PAN -m ignition.msgs.Vector2d -p "x: $dx, y: $dy"
}

goto_position() {
    local x=$1
    local y=$2
    local zoom=${3:-0}
    echo "Going to position ($x, $y) with zoom=$zoom..."
    ign topic -t $TOPIC_SET_VIEW -m ignition.msgs.Pose \
        -p "position: {x: $x, y: $y, z: $zoom}"
}

center_on_gnb() {
    local id=${1:-0}
    echo "Centering view on gNB [$id]..."
    ign topic -t $TOPIC_CENTER_GNB -m ignition.msgs.Int32 -p "data: $id"
}

reset_view() {
    echo "Resetting view to default..."
    ign topic -t $TOPIC_RESET_VIEW -m ignition.msgs.Empty -p ""
}

show_view_info() {
    echo "Current view info:"
    (
        timeout 2 ign topic -e -t $TOPIC_VIEW_INFO -n 1 2>/dev/null | \
            grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
            if [ ! -z "$line" ]; then
                echo "  $line"
            fi
        done
    )
}

# ============================================================================
# Model/Config Functions
# ============================================================================

set_model() {
    local model=$1
    echo "Setting propagation model to: $model"
    ign topic -t $TOPIC_SET_MODEL -m ignition.msgs.StringMsg -p "data: \"$model\""
}

set_config() {
    local config=$1
    echo "Updating configuration: $config"
    ign topic -t $TOPIC_CONFIG -m ignition.msgs.StringMsg -p "data: \"$config\""
}

# ============================================================================
# Query Function (FIXED: start listener BEFORE publishing)
# ============================================================================

query_signal() {
    local x=$1
    local y=$2
    local z=${3:-1.5}
    local result_file="/tmp/query_result_$$.txt"
    
    echo "Querying signal at position: ($x, $y, $z)"
    
    rm -f "$result_file"
    
    # Start listener in background FIRST
    (
        timeout 3 ign topic -e -t $TOPIC_QUERY_RESULT -n 1 2>/dev/null > "$result_file"
    ) &
    local listener_pid=$!
    
    # Wait for listener to be ready
    sleep 0.15
    
    # Now publish the query
    ign topic -t $TOPIC_QUERY_POSITION -m ignition.msgs.Vector3d \
        -p "x: $x, y: $y, z: $z"
    
    # Wait for listener to complete
    wait $listener_pid 2>/dev/null
    
    echo ""
    echo "Query result:"
    
    # Output result
    if [ -s "$result_file" ]; then
        grep -oP 'data: "\K[^"]+' "$result_file" | sed 's/\\n/\n/g'
    else
        echo "  No response received (timeout)"
    fi
    
    rm -f "$result_file"
}

listen_clicks() {
    echo "Listening for heatmap click queries (Ctrl+C to stop)..."
    ign topic -e -t $TOPIC_CLICK_INFO | while read -r line; do
        result=$(echo "$line" | grep -oP 'data: "\K[^"]+')
        if [ ! -z "$result" ]; then
            echo "[$(date '+%H:%M:%S')] $result"
        fi
    done
}

show_status() {
    echo "Current heatmap status:"
    (
        timeout 2 ign topic -e -t $TOPIC_STATUS -n 1 2>/dev/null | \
            grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
            if [ ! -z "$line" ]; then
                echo "  $line"
            fi
        done
    )
}

watch_status() {
    echo "Watching status updates (Ctrl+C to stop)..."
    ign topic -e -t $TOPIC_STATUS | while read -r line; do
        status=$(echo "$line" | grep -oP 'data: "\K[^"]+')
        if [ ! -z "$status" ]; then
            clear
            echo "=== RF Heatmap Status [$(date '+%H:%M:%S')] ==="
            echo ""
            echo "$status" | tr ';' '\n' | while read item; do
                if [ ! -z "$item" ]; then
                    echo "  $item"
                fi
            done
        fi
    done
}

# ============================================================================
# Command Dispatch
# ============================================================================

case "$1" in
    # Multi-gNB commands
    add)
        if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
            echo "Error: X, Y, Z coordinates required"
            echo "Usage: $0 add <x> <y> <z> [name]"
            exit 1
        fi
        add_gnb "$2" "$3" "$4" "$5"
        ;;
    remove)
        if [ -z "$2" ]; then
            echo "Error: gNB ID required"
            exit 1
        fi
        remove_gnb "$2"
        ;;
    move)
        if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ] || [ -z "$5" ]; then
            echo "Error: ID and X, Y, Z coordinates required"
            echo "Usage: $0 move <id> <x> <y> <z>"
            exit 1
        fi
        move_gnb "$2" "$3" "$4" "$5"
        ;;
    enable)
        if [ -z "$2" ]; then echo "Error: gNB ID required"; exit 1; fi
        enable_gnb "$2"
        ;;
    disable)
        if [ -z "$2" ]; then echo "Error: gNB ID required"; exit 1; fi
        disable_gnb "$2"
        ;;
    update)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: ID and parameters required"
            echo "Usage: $0 update <id> <params>"
            echo "Example: $0 update 0 tx_power=35;tx_gain=12"
            exit 1
        fi
        update_gnb "$2" "$3"
        ;;
    list)
        list_gnbs
        ;;
    watch_list)
        watch_gnb_list
        ;;
    combine)
        if [ -z "$2" ]; then
            echo "Error: Mode required (best_server, sum_power, sinr)"
            exit 1
        fi
        set_combine_mode "$2"
        ;;
    
    # Demo commands
    demo_multi)
        demo_multi
        ;;
    demo_interference)
        demo_interference
        ;;
    
    # View controls
    zoom_in)
        zoom_in "${2:-2}"
        ;;
    zoom_out)
        zoom_out "${2:-2}"
        ;;
    zoom)
        if [ -z "$2" ]; then
            echo "Error: Zoom level required"
            exit 1
        fi
        set_zoom "$2"
        ;;
    pan)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: dx and dy required"
            exit 1
        fi
        pan_view "$2" "$3"
        ;;
    goto)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: X and Y coordinates required"
            exit 1
        fi
        goto_position "$2" "$3" "${4:-0}"
        ;;
    center)
        center_on_gnb "${2:-0}"
        ;;
    reset)
        reset_view
        ;;
    view)
        show_view_info
        ;;
    
    # Model/config
    model)
        if [ -z "$2" ]; then
            echo "Error: Model name required"
            exit 1
        fi
        set_model "$2"
        ;;
    config)
        if [ -z "$2" ]; then
            echo "Error: Configuration string required"
            exit 1
        fi
        set_config "$2"
        ;;
    power)
        if [ -z "$2" ]; then echo "Error: Power value required"; exit 1; fi
        set_config "tx_power=$2"
        ;;
    freq)
        if [ -z "$2" ]; then echo "Error: Frequency value required"; exit 1; fi
        set_config "frequency=$2"
        ;;
    wall_loss)
        if [ -z "$2" ]; then echo "Error: Wall loss value required"; exit 1; fi
        set_config "wall_loss=$2"
        ;;
    shadowing)
        case "$2" in
            on|true|1) set_config "shadowing=true" ;;
            off|false|0) set_config "shadowing=false" ;;
            *) echo "Error: Use 'on' or 'off'"; exit 1 ;;
        esac
        ;;
    
    # Interactive
    query)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: X and Y coordinates required"
            exit 1
        fi
        query_signal "$2" "$3" "${4:-1.5}"
        ;;
    listen_clicks)
        listen_clicks
        ;;
    
    # Status
    status)
        show_status
        ;;
    watch_status)
        watch_status
        ;;
    
    help|--help|-h)
        show_help
        ;;
    *)
        echo "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
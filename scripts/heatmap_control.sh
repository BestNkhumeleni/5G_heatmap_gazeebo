#!/bin/bash
# Heatmap Plugin Control Script - Interactive Edition with Pan/Zoom
# Usage: ./heatmap_control.sh <command> [args]

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

show_help() {
    echo "RF Heatmap Plugin Control - Interactive Edition with Pan/Zoom"
    echo ""
    echo "Usage: $0 <command> [arguments]"
    echo ""
    echo "View Control Commands:"
    echo "  zoom_in [amount]     - Zoom in (default: 2 steps)"
    echo "  zoom_out [amount]    - Zoom out (default: 2 steps)"
    echo "  zoom <level>         - Set zoom to specific level (e.g., 2.0 for 2x)"
    echo "  pan <dx> <dy>        - Pan view by dx, dy in world units"
    echo "  goto <x> <y> [zoom]  - Go to position with optional zoom level"
    echo "  center               - Center view on gNB transmitter"
    echo "  reset                - Reset view to default (origin, zoom=1)"
    echo "  view                 - Show current view info"
    echo "  watch_view           - Continuously watch view info (Ctrl+C to stop)"
    echo ""
    echo "Model Commands:"
    echo "  model <name>         - Switch propagation model"
    echo "                         Options: free_space, 3gpp_umi, 3gpp_uma, ray_tracing, hybrid"
    echo ""
    echo "Configuration Commands:"
    echo "  config <params>      - Update configuration parameters"
    echo "                         Format: key1=value1;key2=value2"
    echo ""
    echo "Quick Config Commands:"
    echo "  power <dbm>          - Set transmit power"
    echo "  freq <hz>            - Set frequency"
    echo "  wall_loss <db>       - Set wall penetration loss"
    echo "  shadowing <on|off>   - Enable/disable shadowing"
    echo ""
    echo "Interactive Commands:"
    echo "  move <x> <y> <z>     - Move gNB transmitter to position"
    echo "  query <x> <y> <z>    - Query signal strength at position"
    echo "  listen_clicks        - Listen to click query results"
    echo ""
    echo "Status Commands:"
    echo "  status               - Show current status"
    echo "  watch_status         - Continuously watch status updates"
    echo ""
    echo "Interactive Navigation (keyboard):"
    echo "  interactive          - Start interactive keyboard navigation"
    echo "                         W/S: Pan up/down, A/D: Pan left/right"
    echo "                         +/-: Zoom in/out, C: Center on gNB"
    echo "                         R: Reset view, Q: Quit"
    echo ""
    echo "Examples:"
    echo "  $0 zoom_in 5          # Zoom in 5 steps"
    echo "  $0 pan 10 -5          # Pan 10m right, 5m down"
    echo "  $0 goto 50 30 4       # Go to (50,30) at 4x zoom"
    echo "  $0 center             # Center view on gNB"
    echo "  $0 interactive        # Start keyboard navigation"
}

# View control functions
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
    # Calculate current zoom and send delta to reach target
    # For simplicity, we use set_view instead
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
    echo "Centering view on gNB..."
    ign topic -t $TOPIC_CENTER_GNB -m ignition.msgs.Empty -p ""
}

reset_view() {
    echo "Resetting view to default..."
    ign topic -t $TOPIC_RESET_VIEW -m ignition.msgs.Empty -p ""
}

show_view_info() {
    echo "Current view info:"
    ign topic -e -t $TOPIC_VIEW_INFO -n 1 2>/dev/null | grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
        if [ ! -z "$line" ]; then
            echo "  $line"
        fi
    done
}

watch_view_info() {
    echo "Watching view info (Ctrl+C to stop)..."
    echo ""
    ign topic -e -t $TOPIC_VIEW_INFO | while read -r line; do
        info=$(echo "$line" | grep -oP 'data: "\K[^"]+')
        if [ ! -z "$info" ]; then
            clear
            echo "=== View Info [$(date '+%H:%M:%S')] ==="
            echo ""
            echo "$info" | tr ';' '\n' | while read item; do
                if [ ! -z "$item" ]; then
                    echo "  $item"
                fi
            done
        fi
    done
}

# Interactive keyboard navigation
interactive_mode() {
    echo "=== Interactive Navigation Mode ==="
    echo "Controls:"
    echo "  W/S: Pan up/down"
    echo "  A/D: Pan left/right"
    echo "  +/=: Zoom in"
    echo "  -/_: Zoom out"
    echo "  C: Center on gNB"
    echo "  R: Reset view"
    echo "  V: Show current view"
    echo "  Q: Quit"
    echo ""
    echo "Press keys to navigate (no Enter needed)..."
    
    # Save terminal settings and set to raw mode
    old_stty=$(stty -g)
    stty raw -echo
    
    pan_step=5
    zoom_step=1
    
    while true; do
        char=$(dd bs=1 count=1 2>/dev/null)
        
        case "$char" in
            w|W)
                stty "$old_stty"
                pan_view 0 $pan_step
                stty raw -echo
                ;;
            s|S)
                stty "$old_stty"
                pan_view 0 -$pan_step
                stty raw -echo
                ;;
            a|A)
                stty "$old_stty"
                pan_view -$pan_step 0
                stty raw -echo
                ;;
            d|D)
                stty "$old_stty"
                pan_view $pan_step 0
                stty raw -echo
                ;;
            +|=)
                stty "$old_stty"
                zoom_in $zoom_step
                stty raw -echo
                ;;
            -|_)
                stty "$old_stty"
                zoom_out $zoom_step
                stty raw -echo
                ;;
            c|C)
                stty "$old_stty"
                center_on_gnb
                stty raw -echo
                ;;
            r|R)
                stty "$old_stty"
                reset_view
                stty raw -echo
                ;;
            v|V)
                stty "$old_stty"
                show_view_info
                stty raw -echo
                ;;
            q|Q)
                stty "$old_stty"
                echo ""
                echo "Exiting interactive mode."
                exit 0
                ;;
        esac
    done
}

# Original functions
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

move_gnb() {
    local x=$1
    local y=$2
    local z=$3
    echo "Moving gNB to position: ($x, $y, $z)"
    ign topic -t $TOPIC_SET_POSITION -m ignition.msgs.Pose \
        -p "position: {x: $x, y: $y, z: $z}"
}

query_signal() {
    local x=$1
    local y=$2
    local z=$3
    echo "Querying signal at position: ($x, $y, $z)"
    
    # Start listener in background BEFORE publishing
    ign topic -e -t $TOPIC_QUERY_RESULT -n 1 2>/dev/null > /tmp/query_result.txt &
    listener_pid=$!
    
    # Small delay to ensure subscriber is ready
    sleep 0.1
    
    # Now publish the query
    ign topic -t $TOPIC_QUERY_POSITION -m ignition.msgs.Vector3d \
        -p "x: $x, y: $y, z: $z"
    
    # Wait for listener to receive (with timeout)
    local timeout=3
    local elapsed=0
    while kill -0 $listener_pid 2>/dev/null && [ $elapsed -lt $timeout ]; do
        sleep 0.1
        elapsed=$((elapsed + 1))
    done
    
    # Kill listener if still running (timeout)
    kill $listener_pid 2>/dev/null
    wait $listener_pid 2>/dev/null
    
    echo ""
    echo "Query result:"
    if [ -s /tmp/query_result.txt ]; then
        cat /tmp/query_result.txt | grep -oP 'data: "\K[^"]+' | sed 's/\\n/\n/g'
    else
        echo "  No response received (timeout or plugin not running)"
    fi
    rm -f /tmp/query_result.txt
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
    ign topic -e -t $TOPIC_STATUS -n 1 2>/dev/null | grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
        if [ ! -z "$line" ]; then
            echo "  $line"
        fi
    done
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

# Command dispatch
case "$1" in
    # View controls
    zoom_in)
        zoom_in "${2:-2}"
        ;;
    zoom_out)
        zoom_out "${2:-2}"
        ;;
    zoom)
        if [ -z "$2" ]; then
            echo "Error: Zoom level required (e.g., 2.0 for 2x zoom)"
            exit 1
        fi
        set_zoom "$2"
        ;;
    pan)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: dx and dy required"
            echo "Usage: $0 pan <dx> <dy>"
            exit 1
        fi
        pan_view "$2" "$3"
        ;;
    goto)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: X and Y coordinates required"
            echo "Usage: $0 goto <x> <y> [zoom]"
            exit 1
        fi
        goto_position "$2" "$3" "${4:-0}"
        ;;
    center)
        center_on_gnb
        ;;
    reset)
        reset_view
        ;;
    view)
        show_view_info
        ;;
    watch_view)
        watch_view_info
        ;;
    interactive)
        interactive_mode
        ;;
    # Original commands
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
    move)
        if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
            echo "Error: X, Y, Z coordinates required"
            exit 1
        fi
        move_gnb "$2" "$3" "$4"
        ;;
    query)
        if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
            echo "Error: X, Y, Z coordinates required"
            exit 1
        fi
        query_signal "$2" "$3" "$4"
        ;;
    listen_clicks)
        listen_clicks
        ;;
    status)
        show_status
        ;;
    watch_status)
        watch_status
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
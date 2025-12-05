#!/bin/bash
# Heatmap Plugin Control Script - Interactive Edition
# Usage: ./heatmap_control.sh <command> [args]

TOPIC_SET_MODEL="/gnb/heatmap/set_model"
TOPIC_CONFIG="/gnb/heatmap/config"
TOPIC_STATUS="/gnb/heatmap/status"
TOPIC_SET_POSITION="/gnb/heatmap/set_position"
TOPIC_QUERY_POSITION="/gnb/heatmap/query_position"
TOPIC_CLICK_INFO="/gnb/heatmap/click_info"
TOPIC_QUERY_RESULT="/gnb/heatmap/query_result"

show_help() {
    echo "RF Heatmap Plugin Control - Interactive Edition"
    echo ""
    echo "Usage: $0 <command> [arguments]"
    echo ""
    echo "Model Commands:"
    echo "  model <name>     - Switch propagation model"
    echo "                     Options: free_space, 3gpp_umi, 3gpp_uma, ray_tracing, hybrid"
    echo ""
    echo "Configuration Commands:"
    echo "  config <params>  - Update configuration parameters"
    echo "                     Format: key1=value1;key2=value2"
    echo "                     Keys: tx_power, frequency, tx_height, rx_height,"
    echo "                           wall_loss, reflection_coeff, shadowing, max_reflections"
    echo ""
    echo "Quick Config Commands:"
    echo "  power <dbm>      - Set transmit power"
    echo "  freq <hz>        - Set frequency"
    echo "  wall_loss <db>   - Set wall penetration loss"
    echo "  shadowing <on|off> - Enable/disable shadowing"
    echo ""
    echo "Interactive Commands:"
    echo "  move <x> <y> <z> - Move gNB transmitter to position"
    echo "  query <x> <y> <z> - Query signal strength at position"
    echo "  listen_clicks    - Listen to click query results (Ctrl+C to stop)"
    echo "  listen_queries   - Listen to detailed query results (Ctrl+C to stop)"
    echo ""
    echo "Status Commands:"
    echo "  status           - Show current status"
    echo "  watch_status     - Continuously watch status updates (Ctrl+C to stop)"
    echo ""
    echo "Examples:"
    echo "  $0 model ray_tracing"
    echo "  $0 move 5 10 15"
    echo "  $0 query 10 5 1.5"
    echo "  $0 power 40"
    echo "  $0 listen_clicks"
}

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
    ign topic -t $TOPIC_QUERY_POSITION -m ignition.msgs.Vector3d \
        -p "x: $x, y: $y, z: $z"
    
    # Wait a moment for the query to be processed
    sleep 0.2
    
    # Show the result
    echo ""
    echo "Query result:"
    ign topic -e -t $TOPIC_QUERY_RESULT -n 1 2>/dev/null | grep -oP 'data: "\K[^"]+' | sed 's/\\n/\n/g'
}

listen_clicks() {
    echo "Listening for heatmap click queries (Ctrl+C to stop)..."
    echo "Click on the heatmap or send query commands to see results here."
    echo ""
    ign topic -e -t $TOPIC_CLICK_INFO | while read -r line; do
        result=$(echo "$line" | grep -oP 'data: "\K[^"]+')
        if [ ! -z "$result" ]; then
            echo "[$(date '+%H:%M:%S')] $result"
        fi
    done
}

listen_queries() {
    echo "Listening for detailed query results (Ctrl+C to stop)..."
    echo ""
    ign topic -e -t $TOPIC_QUERY_RESULT | while read -r line; do
        result=$(echo "$line" | grep -oP 'data: "\K[^"]+')
        if [ ! -z "$result" ]; then
            echo "---"
            echo "$result" | sed 's/\\n/\n/g'
            echo ""
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
    echo ""
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

case "$1" in
    model)
        if [ -z "$2" ]; then
            echo "Error: Model name required"
            echo "Options: free_space, 3gpp_umi, 3gpp_uma, ray_tracing, hybrid"
            exit 1
        fi
        set_model "$2"
        ;;
    config)
        if [ -z "$2" ]; then
            echo "Error: Configuration string required"
            echo "Format: key1=value1;key2=value2"
            exit 1
        fi
        set_config "$2"
        ;;
    move)
        if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
            echo "Error: X, Y, Z coordinates required"
            echo "Usage: $0 move <x> <y> <z>"
            exit 1
        fi
        move_gnb "$2" "$3" "$4"
        ;;
    query)
        if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
            echo "Error: X, Y, Z coordinates required"
            echo "Usage: $0 query <x> <y> <z>"
            exit 1
        fi
        query_signal "$2" "$3" "$4"
        ;;
    listen_clicks)
        listen_clicks
        ;;
    listen_queries)
        listen_queries
        ;;
    status)
        show_status
        ;;
    watch_status)
        watch_status
        ;;
    power)
        if [ -z "$2" ]; then
            echo "Error: Power value (dBm) required"
            exit 1
        fi
        set_config "tx_power=$2"
        ;;
    freq)
        if [ -z "$2" ]; then
            echo "Error: Frequency value (Hz) required"
            exit 1
        fi
        set_config "frequency=$2"
        ;;
    wall_loss)
        if [ -z "$2" ]; then
            echo "Error: Wall loss value (dB) required"
            exit 1
        fi
        set_config "wall_loss=$2"
        ;;
    shadowing)
        case "$2" in
            on|true|1)
                set_config "shadowing=true"
                ;;
            off|false|0)
                set_config "shadowing=false"
                ;;
            *)
                echo "Error: Use 'on' or 'off'"
                exit 1
                ;;
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
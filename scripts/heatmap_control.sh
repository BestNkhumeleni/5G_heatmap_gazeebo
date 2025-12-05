#!/bin/bash
# Heatmap Plugin Control Script
# Usage: ./heatmap_control.sh <command> [args]

TOPIC_SET_MODEL="/gnb/heatmap/set_model"
TOPIC_CONFIG="/gnb/heatmap/config"
TOPIC_STATUS="/gnb/heatmap/status"

show_help() {
    echo "RF Heatmap Plugin Control"
    echo ""
    echo "Usage: $0 <command> [arguments]"
    echo ""
    echo "Commands:"
    echo "  model <name>     - Switch propagation model"
    echo "                     Options: free_space, 3gpp_umi, 3gpp_uma, ray_tracing, hybrid"
    echo ""
    echo "  config <params>  - Update configuration parameters"
    echo "                     Format: key1=value1;key2=value2"
    echo "                     Keys: tx_power, frequency, tx_height, rx_height,"
    echo "                           wall_loss, reflection_coeff, shadowing, max_reflections"
    echo ""
    echo "  status           - Show current status"
    echo ""
    echo "  power <dbm>      - Set transmit power (shortcut)"
    echo "  freq <hz>        - Set frequency (shortcut)"
    echo "  wall_loss <db>   - Set wall penetration loss (shortcut)"
    echo "  shadowing <on|off> - Enable/disable shadowing"
    echo ""
    echo "Examples:"
    echo "  $0 model ray_tracing"
    echo "  $0 model 3gpp_umi"
    echo "  $0 config \"tx_power=40;wall_loss=20\""
    echo "  $0 power 40"
    echo "  $0 shadowing on"
    echo "  $0 status"
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

show_status() {
    echo "Current heatmap status:"
    ign topic -e -t $TOPIC_STATUS -n 1 2>/dev/null | grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
        echo "  $line"
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
    status)
        show_status
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
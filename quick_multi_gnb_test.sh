#!/bin/bash
# =============================================================================
# Quick Multi-gNB Test Script
# =============================================================================
# Fast iteration script for testing multi-gNB configurations interactively.
# Collects data at reduced resolution for quick validation.
#
# Usage: ./quick_multi_gnb_test.sh [command] [args]
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"

# =============================================================================
# Topics (same as heatmap_control.sh)
# =============================================================================
TOPIC_SET_MODEL="/gnb/heatmap/set_model"
TOPIC_CONFIG="/gnb/heatmap/config"
TOPIC_STATUS="/gnb/heatmap/status"
TOPIC_QUERY_POSITION="/gnb/heatmap/query_position"
TOPIC_QUERY_RESULT="/gnb/heatmap/query_result"

TOPIC_ADD_GNB="/gnb/heatmap/add_gnb"
TOPIC_REMOVE_GNB="/gnb/heatmap/remove_gnb"
TOPIC_UPDATE_GNB="/gnb/heatmap/update_gnb"
TOPIC_LIST_GNBS="/gnb/heatmap/list_gnbs"
TOPIC_GNB_LIST="/gnb/heatmap/gnb_list"
TOPIC_COMBINE_MODE="/gnb/heatmap/set_combine_mode"

# =============================================================================
# Test Parameters
# =============================================================================
GRID_MIN=-30
GRID_MAX=30
GRID_STEP=5
RX_HEIGHT=1.5
OUTPUT_DIR="${SCRIPT_DIR}/quick_test_results"
TIMESTAMP=$(date +%H%M%S)

# =============================================================================
# Predefined Configurations (ASCII minus signs!)
# =============================================================================
declare -A CONFIGS
CONFIGS["single"]="0,0,12,30,8"
CONFIGS["dual"]="-15,0,12,30,8|15,0,12,30,8"
CONFIGS["triple"]="0,20,12,30,8|-17,-10,12,30,8|17,-10,12,30,8"
CONFIGS["hetero"]="0,0,25,40,12|20,15,8,27,6|-20,15,8,27,6"
CONFIGS["dense"]="-10,10,10,30,8|10,10,10,30,8|-10,-10,10,30,8|10,-10,10,30,8"

# =============================================================================
# Colors
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# =============================================================================
# Utility Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# =============================================================================
# gNB Management Functions (matching heatmap_control.sh style)
# =============================================================================

add_gnb() {
    local x=$1
    local y=$2
    local z=$3
    local name=${4:-""}
    
    ign topic -t $TOPIC_ADD_GNB -m ignition.msgs.Pose \
        -p "name: '$name', position: {x: $x, y: $y, z: $z}" 2>/dev/null
}

remove_gnb() {
    local id=$1
    ign topic -t $TOPIC_REMOVE_GNB -m ignition.msgs.Int32 -p "data: $id" 2>/dev/null
}

update_gnb() {
    local id=$1
    local params=$2
    ign topic -t $TOPIC_UPDATE_GNB -m ignition.msgs.StringMsg \
        -p "data: \"id=$id;$params\"" 2>/dev/null
}

list_gnbs() {
    # Start listener in background FIRST, then send request
    (
        timeout 2 ign topic -e -t $TOPIC_GNB_LIST -n 1 2>/dev/null | \
            grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
            if [ -n "$line" ]; then
                echo "  $line"
            fi
        done
    ) &
    local listener_pid=$!
    
    sleep 0.1
    ign topic -t $TOPIC_LIST_GNBS -m ignition.msgs.Empty -p "" 2>/dev/null
    wait $listener_pid 2>/dev/null
}

set_combine_mode() {
    local mode=$1
    log_info "Setting mode: $mode"
    ign topic -t $TOPIC_COMBINE_MODE -m ignition.msgs.StringMsg \
        -p "data: \"$mode\"" 2>/dev/null
    sleep 0.5
}

set_model() {
    local model=$1
    log_info "Setting model: $model"
    ign topic -t $TOPIC_SET_MODEL -m ignition.msgs.StringMsg \
        -p "data: \"$model\"" 2>/dev/null
    sleep 0.5
}

# =============================================================================
# Query Functions (start listener BEFORE publishing)
# =============================================================================

query_signal() {
    local x=$1
    local y=$2
    local z=${3:-$RX_HEIGHT}
    local result_file="/tmp/query_result_$.txt"
    
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
        -p "x: $x, y: $y, z: $z" 2>/dev/null
    
    # Wait for listener to complete
    wait $listener_pid 2>/dev/null
    
    # Output result
    if [ -s "$result_file" ]; then
        cat "$result_file"
    fi
    
    rm -f "$result_file"
}

query_signal_parsed() {
    local x=$1
    local y=$2
    local z=${3:-$RX_HEIGHT}
    
    local result=$(query_signal "$x" "$y" "$z")
    
    if [ -n "$result" ]; then
        local signal=$(echo "$result" | grep -oP 'Best Signal: \K[-0-9.]+' || echo "-150")
        local gnb=$(echo "$result" | grep -oP 'Best Server: gNB \[\K[0-9]+' || echo "-1")
        local sinr=$(echo "$result" | grep -oP 'SINR: \K[-0-9.]+' || echo "0")
        echo "$signal,$gnb,$sinr"
    else
        echo "-150,-1,0"
    fi
}

show_status() {
    echo "Current heatmap status:"
    ign topic -e -t $TOPIC_STATUS -n 1 2>/dev/null | \
        grep -oP 'data: "\K[^"]+' | tr ';' '\n' | while read line; do
        if [ -n "$line" ]; then
            echo "  $line"
        fi
    done
}

# =============================================================================
# Configuration Setup Functions
# =============================================================================

clear_all_gnbs() {
    log_info "Clearing existing gNBs..."
    for i in $(seq 0 10); do
        remove_gnb "$i"
    done
    sleep 0.3
}

setup_gnbs() {
    local config=$1
    
    if [ -z "${CONFIGS[$config]}" ]; then
        log_error "Unknown configuration: $config"
        echo "Available: ${!CONFIGS[*]}"
        return 1
    fi
    
    clear_all_gnbs
    
    log_info "Setting up configuration: $config"
    local gnb_id=0
    IFS='|' read -ra GNBS <<< "${CONFIGS[$config]}"
    
    for gnb in "${GNBS[@]}"; do
        IFS=',' read -r x y z power gain <<< "$gnb"
        
        log_info "  Adding gNB[$gnb_id] at ($x, $y, $z) power=$power gain=$gain"
        add_gnb "$x" "$y" "$z" "gNB_$gnb_id"
        sleep 0.2
        update_gnb "$gnb_id" "tx_power=$power;tx_gain=$gain"
        
        gnb_id=$((gnb_id + 1))
        sleep 0.3
    done
    
    log_success "Configured $gnb_id gNB(s)"
}

# =============================================================================
# Data Collection Functions
# =============================================================================

test_query() {
    # Quick test to verify queries are working
    log_info "Testing query at (0, 0)..."
    local result=$(query_signal "0" "0")
    
    if [ -n "$result" ]; then
        echo "$result"
        log_success "Query test passed!"
        return 0
    else
        log_error "Query test failed - no response received"
        return 1
    fi
}

collect_quick_data() {
    local output_file=$1
    
    log_info "Collecting quick grid data..."
    
    # First verify queries are working
    log_info "Verifying query system..."
    local test_result=$(query_signal "0" "0")
    if [ -z "$test_result" ]; then
        log_error "Query system not responding. Is Gazebo running with the plugin?"
        return 1
    fi
    log_success "Query system verified"
    
    echo "x,y,best_signal,best_gnb,sinr" > "$output_file"
    
    local count=0
    local total=$(( ((GRID_MAX - GRID_MIN) / GRID_STEP + 1) ** 2 ))
    local success_count=0
    
    for y in $(seq $GRID_MIN $GRID_STEP $GRID_MAX); do
        for x in $(seq $GRID_MIN $GRID_STEP $GRID_MAX); do
            local parsed=$(query_signal_parsed "$x" "$y")
            IFS=',' read -r signal gnb sinr <<< "$parsed"
            
            echo "$x,$y,$signal,$gnb,$sinr" >> "$output_file"
            
            # Track successful queries
            if [ "$signal" != "-150" ]; then
                success_count=$((success_count + 1))
            fi
            
            count=$((count + 1))
            printf "\r  Progress: %d/%d (success: %d)" $count $total $success_count
        done
    done
    
    echo ""
    log_success "Data saved to: $output_file"
    log_info "Successful queries: $success_count / $total"
}

# =============================================================================
# Statistics Functions
# =============================================================================

quick_stats() {
    local file=$1
    
    if [ ! -f "$file" ]; then
        log_warn "File not found: $file"
        return 1
    fi
    
    echo ""
    echo "=== Quick Statistics ==="
    awk -F',' 'NR>1 {
        sum+=$3; count++
        if($3>max || NR==2) max=$3
        if($3<min || NR==2) min=$3
        if($3>=-85) good++
        if($3>=-100) fair++
    } END {
        printf "Points: %d\n", count
        printf "Signal: %.1f dBm (min: %.1f, max: %.1f)\n", sum/count, min, max
        printf "Coverage >= -85dBm: %.1f%%\n", good/count*100
        printf "Coverage >= -100dBm: %.1f%%\n", fair/count*100
    }' "$file"
    echo ""
}

# =============================================================================
# Demo Functions
# =============================================================================

demo_multi() {
    log_info "Setting up multi-gNB demo..."
    
    clear_all_gnbs
    
    add_gnb 0 0 10 "Central"
    sleep 0.3
    add_gnb 25 20 12 "East"
    sleep 0.3
    add_gnb -20 25 10 "West"
    sleep 0.3
    
    log_success "Demo setup complete!"
    echo ""
    echo "Try these commands:"
    echo "  $0 list"
    echo "  $0 mode best_server"
    echo "  $0 mode sinr"
    echo "  $0 query 10 10"
}

# =============================================================================
# Comparison Functions
# =============================================================================

compare_configs() {
    log_info "Running comparison of all configurations..."
    mkdir -p "$OUTPUT_DIR"
    
    local summary_file="$OUTPUT_DIR/comparison_summary.txt"
    echo "=== Multi-gNB Configuration Comparison ===" > "$summary_file"
    echo "Generated: $(date)" >> "$summary_file"
    echo "" >> "$summary_file"
    
    for config in single dual triple hetero dense; do
        echo "--- Configuration: $config ---" >> "$summary_file"
        
        setup_gnbs "$config"
        sleep 1
        
        for mode in best_server sinr; do
            set_combine_mode "$mode"
            sleep 0.5
            
            local output="$OUTPUT_DIR/${config}_${mode}_${TIMESTAMP}.csv"
            collect_quick_data "$output"
            
            echo "Mode: $mode" >> "$summary_file"
            awk -F',' 'NR>1 {
                sum+=$3; count++
                sinr_sum+=$5
            } END {
                printf "  Mean signal: %.1f dBm\n", sum/count
                printf "  Mean SINR: %.1f dB\n", sinr_sum/count
            }' "$output" >> "$summary_file"
        done
        
        echo "" >> "$summary_file"
    done
    
    log_success "Comparison complete!"
    cat "$summary_file"
}

# =============================================================================
# Interactive Mode
# =============================================================================

interactive_test() {
    echo ""
    echo "=== Interactive Multi-gNB Test ==="
    echo ""
    echo "Available configurations:"
    echo "  single  - Single central gNB"
    echo "  dual    - Two symmetric gNBs"
    echo "  triple  - Three gNBs in triangle"
    echo "  hetero  - Macro + small cells"
    echo "  dense   - Four-corner deployment"
    echo ""
    echo "Commands:"
    echo "  config <name>  - Switch configuration"
    echo "  mode <mode>    - Set mode (best_server/sinr/sum_power)"
    echo "  model <name>   - Set propagation model"
    echo "  test           - Run quick data collection"
    echo "  stats          - Show statistics from last test"
    echo "  compare        - Run all configurations"
    echo "  query <x> <y>  - Query single point"
    echo "  list           - List current gNBs"
    echo "  status         - Show system status"
    echo "  demo           - Run demo setup"
    echo "  quit           - Exit"
    echo ""
    
    local last_output=""
    
    while true; do
        read -p "> " cmd args
        
        case "$cmd" in
            config)
                setup_gnbs "$args"
                ;;
            mode)
                set_combine_mode "$args"
                ;;
            model)
                set_model "$args"
                ;;
            test)
                mkdir -p "$OUTPUT_DIR"
                last_output="$OUTPUT_DIR/quick_${TIMESTAMP}.csv"
                collect_quick_data "$last_output"
                quick_stats "$last_output"
                ;;
            testq)
                test_query
                ;;
            stats)
                if [ -n "$last_output" ]; then
                    quick_stats "$last_output"
                else
                    log_warn "No test data yet. Run 'test' first."
                fi
                ;;
            compare)
                compare_configs
                ;;
            query)
                local qx=$(echo "$args" | cut -d' ' -f1)
                local qy=$(echo "$args" | cut -d' ' -f2)
                log_info "Querying ($qx, $qy)..."
                query_signal "$qx" "$qy"
                ;;
            list)
                echo "Current gNBs:"
                list_gnbs
                ;;
            status)
                show_status
                ;;
            demo)
                demo_multi
                ;;
            quit|exit|q)
                echo "Goodbye!"
                exit 0
                ;;
            help|h|\?)
                echo "Commands: config, mode, model, test, testq, stats, compare, query, list, status, demo, quit"
                ;;
            "")
                # Empty input, do nothing
                ;;
            *)
                log_warn "Unknown command: $cmd (type 'help' for commands)"
                ;;
        esac
    done
}

# =============================================================================
# Help
# =============================================================================

show_help() {
    cat << EOF
Quick Multi-gNB Test Script

Usage: $0 [command] [args]

Commands:
  interactive         Start interactive testing mode (default)
  config <name>       Set up a configuration (single/dual/triple/hetero/dense)
  test [config]       Run quick test on a configuration
  compare             Compare all configurations
  query <x> <y>       Query signal at position
  list                List current gNBs
  status              Show system status
  demo                Set up demo gNBs
  help                Show this help

Examples:
  $0                      # Start interactive mode
  $0 config dual          # Set up dual gNB configuration
  $0 test triple          # Quick test of triple configuration
  $0 query 10 15          # Query signal at (10, 15)
  $0 compare              # Compare all configurations

Note: Gazebo must be running with the heatmap plugin loaded.
      Start with: ign gazebo worlds/scenario_open_field.sdf
EOF
}

# =============================================================================
# Command Dispatch (matching heatmap_control.sh style)
# =============================================================================

case "${1:-interactive}" in
    interactive|i)
        interactive_test
        ;;
    config)
        if [ -z "$2" ]; then
            echo "Available configurations: ${!CONFIGS[*]}"
            exit 1
        fi
        setup_gnbs "$2"
        ;;
    test)
        config="${2:-single}"
        setup_gnbs "$config"
        sleep 1
        mkdir -p "$OUTPUT_DIR"
        output="$OUTPUT_DIR/${config}_test_${TIMESTAMP}.csv"
        collect_quick_data "$output"
        quick_stats "$output"
        ;;
    testq)
        test_query
        ;;
    compare)
        compare_configs
        ;;
    query)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Error: X and Y coordinates required"
            echo "Usage: $0 query <x> <y>"
            exit 1
        fi
        echo "Querying ($2, $3)..."
        query_signal "$2" "$3"
        ;;
    list)
        echo "Current gNBs:"
        list_gnbs
        ;;
    status)
        show_status
        ;;
    mode)
        if [ -z "$2" ]; then
            echo "Error: Mode required (best_server/sum_power/sinr)"
            exit 1
        fi
        set_combine_mode "$2"
        ;;
    model)
        if [ -z "$2" ]; then
            echo "Error: Model required (free_space/3gpp_umi/3gpp_uma/ray_tracing/hybrid)"
            exit 1
        fi
        set_model "$2"
        ;;
    demo)
        demo_multi
        ;;
    help|-h|--help)
        show_help
        ;;
    *)
        log_warn "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
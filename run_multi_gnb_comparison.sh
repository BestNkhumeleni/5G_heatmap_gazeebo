#!/bin/bash
# =============================================================================
# Multi-gNB Deployment Comparison Script (FIXED)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLDS_DIR="${SCRIPT_DIR}/worlds"
CONTROL_SCRIPT="${SCRIPT_DIR}/heatmap_control.sh"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
OUTPUT_DIR="${SCRIPT_DIR}/multi_gnb_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR="${OUTPUT_DIR}/${TIMESTAMP}"

# =============================================================================
# Topics
# =============================================================================
TOPIC_SET_MODEL="/gnb/heatmap/set_model"
TOPIC_QUERY_POSITION="/gnb/heatmap/query_position"
TOPIC_QUERY_RESULT="/gnb/heatmap/query_result"
TOPIC_ADD_GNB="/gnb/heatmap/add_gnb"
TOPIC_REMOVE_GNB="/gnb/heatmap/remove_gnb"
TOPIC_UPDATE_GNB="/gnb/heatmap/update_gnb"
TOPIC_LIST_GNBS="/gnb/heatmap/list_gnbs"
TOPIC_GNB_LIST="/gnb/heatmap/gnb_list"
TOPIC_COMBINE_MODE="/gnb/heatmap/set_combine_mode"

# =============================================================================
# Deployment Configurations (ASCII minus signs)
# =============================================================================
DEPLOYMENTS=(
    "single_central:Single Central gNB:0,0,12,30,8"
    "single_elevated:Single Elevated gNB:0,0,25,33,10"
    "dual_symmetric:Two gNBs Symmetric:-25,0,12,30,8|25,0,12,30,8"
    "dual_diagonal:Two gNBs Diagonal:-20,-20,12,30,8|20,20,12,30,8"
    "dual_heterogeneous:Macro + Small Cell:0,0,25,40,12|20,15,8,27,6"
    "tri_triangle:Triangle Pattern:0,25,12,30,8|-22,-12,12,30,8|22,-12,12,30,8"
    "tri_linear:Linear Array:-30,0,12,30,8|0,0,12,30,8|30,0,12,30,8"
    "tri_heterogeneous:Macro + 2 Small Cells:0,0,25,40,12|-25,20,8,27,6|25,-20,8,27,6"
    "penta_cross:Cross Pattern:0,0,12,30,8|0,30,12,30,8|0,-30,12,30,8|30,0,12,30,8|-30,0,12,30,8"
    "penta_pentagon:Pentagon Pattern:0,32,12,30,8|30,10,12,30,8|19,-26,12,30,8|-19,-26,12,30,8|-30,10,12,30,8"
)

COMBINE_MODES=("best_server" "sum_power" "sinr")
MODELS=("3gpp_umi" "ray_tracing" "hybrid")

# Test grid parameters
GRID_MIN_X=-40
GRID_MAX_X=40
GRID_MIN_Y=-40
GRID_MAX_Y=40
GRID_STEP=8.0
RX_HEIGHT=1.5

# Timing
STARTUP_WAIT=6
MODEL_SWITCH_WAIT=2
GNB_SETUP_WAIT=1

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# =============================================================================
# Utility Functions
# =============================================================================

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_section() { echo -e "\n${CYAN}=== $1 ===${NC}\n"; }

check_dependencies() {
    log_info "Checking dependencies..."
    
    local missing=0
    for cmd in ign bc python3; do
        if ! command -v $cmd &> /dev/null; then
            log_error "$cmd not found"
            missing=1
        fi
    done
    
    if [ $missing -eq 1 ]; then
        exit 1
    fi
    
    log_success "All dependencies found"
}

setup_output_dir() {
    log_info "Setting up output directory: ${RESULTS_DIR}"
    mkdir -p "${RESULTS_DIR}"/{raw_data,figures,heatmaps,analysis}
}

# =============================================================================
# Gazebo Control Functions
# =============================================================================

start_gazebo() {
    local world_file=$1
    local world_path="${WORLDS_DIR}/${world_file}.sdf"
    
    if [ ! -f "$world_path" ]; then
        log_error "World file not found: $world_path"
        return 1
    fi
    
    log_info "Starting Gazebo with world: $world_file"
    ign gazebo -s -r "$world_path" &
    GAZEBO_PID=$!
    
    sleep $STARTUP_WAIT
    
    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        log_error "Gazebo failed to start"
        return 1
    fi
    
    log_success "Gazebo started (PID: $GAZEBO_PID)"
    return 0
}

stop_gazebo() {
    if [ -n "$GAZEBO_PID" ]; then
        log_info "Stopping Gazebo (PID: $GAZEBO_PID)..."
        kill $GAZEBO_PID 2>/dev/null || true
        
        local wait_count=0
        while kill -0 $GAZEBO_PID 2>/dev/null && [ $wait_count -lt 50 ]; do
            sleep 0.1
            wait_count=$((wait_count + 1))
        done
        
        if kill -0 $GAZEBO_PID 2>/dev/null; then
            log_warn "Gazebo didn't stop gracefully, force killing..."
            kill -9 $GAZEBO_PID 2>/dev/null || true
            sleep 0.5
        fi
        
        pkill -9 -f "ign gazebo" 2>/dev/null || true
        pkill -9 -f "ruby.*ign" 2>/dev/null || true
        
        GAZEBO_PID=""
        sleep 1
        log_success "Gazebo stopped"
    fi
    
    return 0
}

# =============================================================================
# gNB Management Functions
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

clear_all_gnbs() {
    log_info "Clearing existing gNBs..."
    for i in $(seq 0 10); do
        remove_gnb "$i"
    done
    sleep 0.5
}

setup_gnbs() {
    local gnb_specs=$1
    
    clear_all_gnbs
    
    log_info "Setting up gNBs: $gnb_specs"
    
    local gnb_id=0
    IFS='|' read -ra GNBS <<< "$gnb_specs"
    
    for gnb in "${GNBS[@]}"; do
        IFS=',' read -r x y z power gain <<< "$gnb"
        
        log_info "  Adding gNB[$gnb_id] at ($x, $y, $z) power=$power gain=$gain"
        
        add_gnb "$x" "$y" "$z" "gNB_$gnb_id"
        sleep 0.2
        update_gnb "$gnb_id" "tx_power=$power;tx_gain=$gain"
        
        gnb_id=$((gnb_id + 1))
        sleep $GNB_SETUP_WAIT
    done
    
    log_success "Set up $gnb_id gNB(s)"
    return 0
}

set_model() {
    local model=$1
    log_info "Switching to model: $model"
    ign topic -t $TOPIC_SET_MODEL -m ignition.msgs.StringMsg \
        -p "data: \"$model\"" 2>/dev/null
    sleep $MODEL_SWITCH_WAIT
}

set_combine_mode() {
    local mode=$1
    log_info "Setting combine mode: $mode"
    ign topic -t $TOPIC_COMBINE_MODE -m ignition.msgs.StringMsg \
        -p "data: \"$mode\"" 2>/dev/null
    sleep 0.5
}

# =============================================================================
# Query Function (FIXED: start listener BEFORE publishing)
# =============================================================================

query_signal() {
    local x=$1
    local y=$2
    local z=$3
    local result_file="/tmp/multi_gnb_query_$$.txt"
    
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
    else
        echo "TIMEOUT"
    fi
    
    rm -f "$result_file"
}

# =============================================================================
# Data Collection Functions
# =============================================================================

collect_grid_data() {
    local deployment_name=$1
    local model=$2
    local combine_mode=$3
    local output_file=$4
    
    log_info "Collecting grid data for $deployment_name / $model / $combine_mode"
    
    echo "x,y,z,best_signal_dbm,best_gnb_id,total_power_dbm,sinr_db,num_gnbs_visible" > "$output_file"
    
    local total_points=$(echo "(($GRID_MAX_X - $GRID_MIN_X) / $GRID_STEP + 1) * (($GRID_MAX_Y - $GRID_MIN_Y) / $GRID_STEP + 1)" | bc)
    local point_count=0
    
    for y in $(seq $GRID_MIN_Y $GRID_STEP $GRID_MAX_Y); do
        for x in $(seq $GRID_MIN_X $GRID_STEP $GRID_MAX_X); do
            point_count=$((point_count + 1))
            
            local result=$(query_signal "$x" "$y" "$RX_HEIGHT")
            
            if [ "$result" != "TIMEOUT" ]; then
                local best_signal=$(echo "$result" | grep -oP 'Best Signal: \K[-0-9.]+' || echo "-150")
                local best_gnb=$(echo "$result" | grep -oP 'Best Server: gNB \[\K[0-9]+' || echo "-1")
                local total_power=$(echo "$result" | grep -oP 'Total Power: \K[-0-9.]+' || echo "-150")
                local sinr=$(echo "$result" | grep -oP 'SINR: \K[-0-9.]+' || echo "0")
                local num_visible=$(echo "$result" | grep -c '\[.*\].*dBm' || echo "0")
                
                echo "$x,$y,$RX_HEIGHT,$best_signal,$best_gnb,$total_power,$sinr,$num_visible" >> "$output_file"
            else
                echo "$x,$y,$RX_HEIGHT,-150,-1,-150,0,0" >> "$output_file"
            fi
            
            if (( point_count % 10 == 0 )); then
                printf "\r  Progress: %d/%d (%.1f%%)" $point_count $total_points $(echo "scale=1; $point_count * 100 / $total_points" | bc)
            fi
        done
    done
    
    printf "\r  Progress: %d/%d (100.0%%)\n" $total_points $total_points
    log_success "Data saved to: $output_file"
}

collect_radial_data() {
    local deployment_name=$1
    local model=$2
    local output_file=$3
    local center_x=${4:-0}
    local center_y=${5:-0}
    
    log_info "Collecting radial data from center ($center_x, $center_y)"
    
    echo "angle_deg,distance_m,x,y,best_signal_dbm,best_gnb_id,sinr_db" > "$output_file"
    
    for angle in $(seq 0 15 345); do
        for dist in $(seq 5 5 60); do
            local rad=$(echo "scale=6; $angle * 3.14159265 / 180" | bc -l)
            local x=$(echo "scale=2; $center_x + $dist * c($rad)" | bc -l)
            local y=$(echo "scale=2; $center_y + $dist * s($rad)" | bc -l)
            
            local result=$(query_signal "$x" "$y" "$RX_HEIGHT")
            
            if [ "$result" != "TIMEOUT" ]; then
                local best_signal=$(echo "$result" | grep -oP 'Best Signal: \K[-0-9.]+' || echo "-150")
                local best_gnb=$(echo "$result" | grep -oP 'Best Server: gNB \[\K[0-9]+' || echo "-1")
                local sinr=$(echo "$result" | grep -oP 'SINR: \K[-0-9.]+' || echo "0")
                
                echo "$angle,$dist,$x,$y,$best_signal,$best_gnb,$sinr" >> "$output_file"
            fi
        done
        printf "."
    done
    echo ""
    log_success "Radial data saved"
}

collect_cell_boundary_data() {
    local deployment_name=$1
    local model=$2
    local output_file=$3
    
    log_info "Collecting cell boundary data"
    
    echo "x,y,serving_gnb,signal_dbm,next_gnb,next_signal_dbm,margin_db" > "$output_file"
    
    for y in $(seq $GRID_MIN_Y 2.0 $GRID_MAX_Y); do
        for x in $(seq $GRID_MIN_X 2.0 $GRID_MAX_X); do
            local result=$(query_signal "$x" "$y" "$RX_HEIGHT")
            
            if [ "$result" != "TIMEOUT" ]; then
                local signals=$(echo "$result" | grep -oP '\[\K[0-9]+\].*?[-0-9.]+(?= dBm)' | head -2)
                
                if [ -n "$signals" ]; then
                    local first=$(echo "$signals" | head -1)
                    local second=$(echo "$signals" | tail -1)
                    
                    local gnb1=$(echo "$first" | grep -oP '^\d+')
                    local sig1=$(echo "$first" | grep -oP '[-0-9.]+$')
                    local gnb2=$(echo "$second" | grep -oP '^\d+')
                    local sig2=$(echo "$second" | grep -oP '[-0-9.]+$')
                    
                    if [ -n "$sig1" ] && [ -n "$sig2" ]; then
                        local margin=$(echo "scale=2; $sig1 - $sig2" | bc)
                        
                        if (( $(echo "$margin < 6" | bc -l) )); then
                            echo "$x,$y,$gnb1,$sig1,$gnb2,$sig2,$margin" >> "$output_file"
                        fi
                    fi
                fi
            fi
        done
    done
    
    log_success "Cell boundary data saved"
}

# =============================================================================
# Analysis Functions
# =============================================================================

calculate_coverage_stats() {
    local csv_file=$1
    local output_file=$2
    
    awk -F',' '
    BEGIN { OFS="," }
    NR > 1 {
        signal = $4
        sinr = $7
        
        total++
        sum_signal += signal
        sum_sinr += sinr
        
        if (signal >= -70) excellent++
        else if (signal >= -85) good++
        else if (signal >= -100) fair++
        else if (signal >= -115) poor++
        else no_coverage++
        
        if (sinr >= 20) sinr_excellent++
        else if (sinr >= 10) sinr_good++
        else if (sinr >= 0) sinr_fair++
        else sinr_poor++
        
        if (signal > max_signal || NR == 2) max_signal = signal
        if (signal < min_signal || NR == 2) min_signal = signal
    }
    END {
        print "metric,value"
        print "total_points," total
        print "mean_signal_dbm," sum_signal/total
        print "mean_sinr_db," sum_sinr/total
        print "max_signal_dbm," max_signal
        print "min_signal_dbm," min_signal
        print "coverage_excellent_pct," excellent/total*100
        print "coverage_good_pct," good/total*100
        print "coverage_fair_pct," fair/total*100
        print "coverage_poor_pct," poor/total*100
        print "coverage_none_pct," no_coverage/total*100
        print "sinr_excellent_pct," sinr_excellent/total*100
        print "sinr_good_pct," sinr_good/total*100
        print "sinr_fair_pct," sinr_fair/total*100
        print "sinr_poor_pct," sinr_poor/total*100
    }
    ' "$csv_file" > "$output_file"
}

# =============================================================================
# Main Experiment Functions
# =============================================================================

run_deployment_comparison() {
    local scenario=$1
    
    log_section "Running Multi-gNB Deployment Comparison"
    log_info "Scenario: $scenario"
    
    if ! start_gazebo "$scenario"; then
        log_error "Failed to start Gazebo"
        return 1
    fi
    
    for deployment in "${DEPLOYMENTS[@]}"; do
        IFS=':' read -r dep_name dep_desc gnb_specs <<< "$deployment"
        
        log_section "Deployment: $dep_desc ($dep_name)"
        
        for model in "${MODELS[@]}"; do
            log_info "Model: $model"
            
            setup_gnbs "$gnb_specs"
            set_model "$model"
            
            for mode in "${COMBINE_MODES[@]}"; do
                set_combine_mode "$mode"
                
                local output_file="${RESULTS_DIR}/raw_data/${scenario}_${dep_name}_${model}_${mode}.csv"
                collect_grid_data "$dep_name" "$model" "$mode" "$output_file"
                
                local stats_file="${RESULTS_DIR}/raw_data/${scenario}_${dep_name}_${model}_${mode}_stats.csv"
                calculate_coverage_stats "$output_file" "$stats_file"
            done
        done
    done
    
    stop_gazebo
    return 0
}

run_interference_analysis() {
    local scenario=$1
    
    log_section "Running Interference Analysis"
    
    if ! start_gazebo "$scenario"; then
        log_warn "Failed to start Gazebo for interference analysis, skipping..."
        return 1
    fi
    
    local separations=(10 15 20 25 30 40 50)
    local output_file="${RESULTS_DIR}/raw_data/interference_vs_separation.csv"
    
    echo "separation_m,mean_sinr_db,min_sinr_db,interference_zone_pct" > "$output_file"
    
    for sep in "${separations[@]}"; do
        log_info "Testing separation: ${sep}m"
        
        local half_sep=$(echo "scale=2; $sep / 2" | bc)
        setup_gnbs "-${half_sep},0,12,30,8|${half_sep},0,12,30,8"
        set_model "3gpp_umi"
        set_combine_mode "sinr"
        
        sleep 2
        
        local sinr_sum=0
        local sinr_min=100
        local interference_count=0
        local total=0
        
        for x in $(seq -$half_sep 2 $half_sep); do
            local result=$(query_signal "$x" "0" "$RX_HEIGHT")
            
            if [ "$result" != "TIMEOUT" ]; then
                local sinr=$(echo "$result" | grep -oP 'SINR: \K[-0-9.]+' || echo "0")
                
                sinr_sum=$(echo "scale=2; $sinr_sum + $sinr" | bc)
                if (( $(echo "$sinr < $sinr_min" | bc -l) )); then
                    sinr_min=$sinr
                fi
                if (( $(echo "$sinr < 5" | bc -l) )); then
                    interference_count=$((interference_count + 1))
                fi
                total=$((total + 1))
            fi
        done
        
        if [ $total -gt 0 ]; then
            local mean_sinr=$(echo "scale=2; $sinr_sum / $total" | bc)
            local interf_pct=$(echo "scale=2; $interference_count * 100 / $total" | bc)
            echo "$sep,$mean_sinr,$sinr_min,$interf_pct" >> "$output_file"
        fi
    done
    
    stop_gazebo
    log_success "Interference analysis complete"
    return 0
}

run_coverage_complementarity() {
    local scenario=$1
    
    log_section "Running Coverage Complementarity Analysis"
    
    if ! start_gazebo "$scenario"; then
        log_warn "Failed to start Gazebo for complementarity analysis, skipping..."
        return 1
    fi
    
    local output_file="${RESULTS_DIR}/raw_data/coverage_complementarity.csv"
    echo "config,num_gnbs,coverage_total_pct,coverage_gain_pct,overlap_pct" > "$output_file"
    
    local configs=(
        "single:0,0,12,30,8"
        "dual:0,0,12,30,8|25,0,12,30,8"
        "triple:0,0,12,30,8|25,0,12,30,8|12.5,22,12,30,8"
    )
    
    local prev_coverage=0
    
    for config in "${configs[@]}"; do
        IFS=':' read -r name specs <<< "$config"
        local num_gnbs=$(echo "$specs" | tr '|' '\n' | wc -l)
        
        log_info "Testing: $name ($num_gnbs gNBs)"
        
        setup_gnbs "$specs"
        set_model "3gpp_umi"
        set_combine_mode "best_server"
        
        sleep 2
        
        local covered=0
        local total=0
        local overlap_count=0
        
        for y in $(seq -30 5 30); do
            for x in $(seq -30 5 30); do
                local result=$(query_signal "$x" "$y" "$RX_HEIGHT")
                
                if [ "$result" != "TIMEOUT" ]; then
                    local best_signal=$(echo "$result" | grep -oP 'Best Signal: \K[-0-9.]+' || echo "-150")
                    local num_visible=$(echo "$result" | grep -c '\[.*\].*dBm' || echo "0")
                    
                    total=$((total + 1))
                    if (( $(echo "$best_signal >= -100" | bc -l) )); then
                        covered=$((covered + 1))
                    fi
                    if (( num_visible > 1 )); then
                        overlap_count=$((overlap_count + 1))
                    fi
                fi
            done
        done
        
        if [ $total -gt 0 ]; then
            local coverage_pct=$(echo "scale=2; $covered * 100 / $total" | bc)
            local gain=$(echo "scale=2; $coverage_pct - $prev_coverage" | bc)
            local overlap_pct=$(echo "scale=2; $overlap_count * 100 / $total" | bc)
            
            echo "$name,$num_gnbs,$coverage_pct,$gain,$overlap_pct" >> "$output_file"
            prev_coverage=$coverage_pct
        fi
    done
    
    stop_gazebo
    log_success "Complementarity analysis complete"
    return 0
}

# =============================================================================
# Report Generation
# =============================================================================

generate_summary_report() {
    local report_file="${RESULTS_DIR}/multi_gnb_report.txt"
    
    log_info "Generating summary report..."
    
    cat > "$report_file" << 'EOF'
================================================================================
        MULTI-gNB DEPLOYMENT COMPARISON REPORT
        For Academic Paper: Single vs Multiple gNB Analysis
================================================================================

EOF
    
    echo "Report generated: $(date)" >> "$report_file"
    echo "Output directory: ${RESULTS_DIR}" >> "$report_file"
    echo "" >> "$report_file"
    
    for csv in "${RESULTS_DIR}"/raw_data/*_stats.csv; do
        if [ -f "$csv" ]; then
            local basename=$(basename "$csv" _stats.csv)
            echo "--- $basename ---" >> "$report_file"
            cat "$csv" >> "$report_file"
            echo "" >> "$report_file"
        fi
    done
    
    log_success "Report saved to: $report_file"
}

generate_latex_tables() {
    local latex_file="${RESULTS_DIR}/latex_multi_gnb_tables.tex"
    
    cat > "$latex_file" << 'EOF'
% Multi-gNB Deployment Comparison Tables
% Auto-generated for conference paper

\begin{table*}[htbp]
\centering
\caption{Single vs Multi-gNB Deployment Coverage Comparison}
\label{tab:multi_gnb_coverage}
\begin{tabular}{lcrrrrrr}
\toprule
\textbf{Deployment} & \textbf{gNBs} & \textbf{Mean} & \textbf{Coverage} & \textbf{Mean} & \textbf{Overlap} & \textbf{95\%} \\
 & & \textbf{RSRP (dBm)} & \textbf{(\%)} & \textbf{SINR (dB)} & \textbf{(\%)} & \textbf{Coverage (m)} \\
\midrule
% Data to be filled from analysis
\bottomrule
\end{tabular}
\end{table*}

\begin{table}[htbp]
\centering
\caption{Interference Analysis: SINR vs gNB Separation}
\label{tab:interference_separation}
\begin{tabular}{rrrr}
\toprule
\textbf{Separation} & \textbf{Mean SINR} & \textbf{Min SINR} & \textbf{Interference} \\
\textbf{(m)} & \textbf{(dB)} & \textbf{(dB)} & \textbf{Zone (\%)} \\
\midrule
% Data from interference_vs_separation.csv
\bottomrule
\end{tabular}
\end{table}
EOF
    
    log_success "LaTeX tables saved to: $latex_file"
}

# =============================================================================
# Cleanup and Main
# =============================================================================

cleanup() {
    stop_gazebo
    rm -f /tmp/multi_gnb_query_$$.txt 2>/dev/null || true
}
trap cleanup EXIT

print_usage() {
    cat << EOF
Usage: $0 [OPTIONS] [COMMAND]

Commands:
  all                     Run all experiments
  deployment <scenario>   Run deployment comparison
  interference <scenario> Run interference analysis
  complementarity <scenario> Run coverage complementarity
  report                  Generate reports from existing data

Options:
  -h, --help             Show this help
  -o, --output DIR       Set output directory
  
Scenarios:
  scenario_open_field    Open environment (baseline)
  scenario_dense_urban   Dense urban with buildings
  scenario_multi_gnb     Pre-configured multi-gNB scenario

Examples:
  $0 all scenario_open_field
  $0 deployment scenario_dense_urban
  $0 interference scenario_multi_gnb
EOF
}

main() {
    echo ""
    echo "========================================================"
    echo "  Multi-gNB Deployment Comparison Tool"
    echo "  For 5G Network Planning Research"
    echo "========================================================"
    echo ""
    
    check_dependencies
    setup_output_dir
    
    local command="${1:-all}"
    local scenario="${2:-scenario_open_field}"
    
    case "$command" in
        all)
            run_deployment_comparison "$scenario"
            run_interference_analysis "$scenario"
            run_coverage_complementarity "$scenario"
            generate_summary_report
            generate_latex_tables
            ;;
        deployment)
            run_deployment_comparison "$scenario"
            ;;
        interference)
            run_interference_analysis "$scenario"
            ;;
        complementarity)
            run_coverage_complementarity "$scenario"
            ;;
        report)
            generate_summary_report
            generate_latex_tables
            ;;
        -h|--help|help)
            print_usage
            ;;
        *)
            log_error "Unknown command: $command"
            print_usage
            exit 1
            ;;
    esac
    
    echo ""
    log_success "Experiment complete!"
    log_info "Results saved to: ${RESULTS_DIR}"
}

main "$@"
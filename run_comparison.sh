#!/bin/bash
# =============================================================================
# RF Propagation Model Comparison Script
# =============================================================================
# This script runs all propagation models across all scenarios and generates
# a comprehensive comparison report with statistics.
# =============================================================================

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLDS_DIR="${SCRIPT_DIR}/worlds"

# Set plugin path so Gazebo can find HeatmapPlugin
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
OUTPUT_DIR="${SCRIPT_DIR}/comparison_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR="${OUTPUT_DIR}/${TIMESTAMP}"

# Scenarios to test
SCENARIOS=(
    "scenario_open_field:Open Field (Rural)"
    "scenario_dense_urban:Dense Urban"
    "scenario_street_canyon:Street Canyon (UMi)"
    "scenario_elevated_macro:Elevated Macro (UMa)"
    "scenario_indoor_outdoor:Indoor-Outdoor"
)

# Propagation models to compare
MODELS=(
    "free_space:Free Space Path Loss"
    "3gpp_umi:3GPP TR 38.901 UMi"
    "3gpp_uma:3GPP TR 38.901 UMa"
    "ray_tracing:Ray Tracing"
    "hybrid:Hybrid (3GPP + RT)"
)

# Test points (relative to gNB at origin unless otherwise specified)
# Format: "x,y,z:description"
TEST_POINTS=(
    "5,0,1.5:5m LOS"
    "10,0,1.5:10m LOS"
    "20,0,1.5:20m LOS"
    "30,0,1.5:30m LOS"
    "50,0,1.5:50m LOS"
    "10,10,1.5:14m diagonal"
    "20,20,1.5:28m diagonal"
    "-10,15,1.5:18m behind"
    "25,-15,1.5:29m offset"
    "40,30,1.5:50m far diagonal"
)

# Signal quality thresholds (dBm)
EXCELLENT_THRESHOLD=-70
GOOD_THRESHOLD=-85
FAIR_THRESHOLD=-100
POOR_THRESHOLD=-115

# Timing
STARTUP_WAIT=5
MODEL_SWITCH_WAIT=2
QUERY_TIMEOUT=3

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Temp file for query results (use PID to avoid conflicts)
QUERY_RESULT_FILE="/tmp/query_result_$$.txt"

# =============================================================================
# Utility Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
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

check_dependencies() {
    log_info "Checking dependencies..."
    
    if ! command -v ign &> /dev/null; then
        log_error "Ignition Gazebo not found. Please install ignition-gazebo6."
        exit 1
    fi
    
    if ! command -v bc &> /dev/null; then
        log_error "bc calculator not found. Please install bc."
        exit 1
    fi
    
    log_success "All dependencies found."
}

setup_output_dir() {
    log_info "Setting up output directory: ${RESULTS_DIR}"
    mkdir -p "${RESULTS_DIR}"
    mkdir -p "${RESULTS_DIR}/raw_data"
    mkdir -p "${RESULTS_DIR}/heatmaps"
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
    
    # Start Gazebo in background (headless mode for automated testing)
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
    if [ ! -z "$GAZEBO_PID" ]; then
        log_info "Stopping Gazebo (PID: $GAZEBO_PID)..."
        kill $GAZEBO_PID 2>/dev/null || true
        wait $GAZEBO_PID 2>/dev/null || true
        GAZEBO_PID=""
        sleep 1
        log_success "Gazebo stopped"
    fi
}

switch_model() {
    local model=$1
    log_info "Switching to propagation model: $model"
    ign topic -t /gnb/heatmap/set_model -m ignition.msgs.StringMsg -p "data: \"$model\"" 2>/dev/null
    sleep $MODEL_SWITCH_WAIT
}

query_signal() {
    local x=$1
    local y=$2
    local z=$3
    
    # Clean up any previous result file
    rm -f "$QUERY_RESULT_FILE"
    
    # Start listener in background BEFORE publishing
    ign topic -e -t /gnb/heatmap/query_result -n 1 2>/dev/null > "$QUERY_RESULT_FILE" &
    local listener_pid=$!
    
    # Small delay to ensure subscriber is ready
    sleep 0.1
    
    # Now publish the query
    ign topic -t /gnb/heatmap/query_position -m ignition.msgs.Vector3d \
        -p "x: $x, y: $y, z: $z" 2>/dev/null
    
    # Wait for listener to receive (with timeout)
    local elapsed=0
    while kill -0 $listener_pid 2>/dev/null && [ $elapsed -lt $QUERY_TIMEOUT ]; do
        sleep 0.1
        elapsed=$((elapsed + 1))
    done
    
    # Kill listener if still running (timeout)
    kill $listener_pid 2>/dev/null || true
    wait $listener_pid 2>/dev/null || true
    
    # Extract signal strength from result
    local result="-999"
    if [ -s "$QUERY_RESULT_FILE" ]; then
        result=$(grep -oP 'Signal Strength: \K[-0-9.]+' "$QUERY_RESULT_FILE" 2>/dev/null || echo "-999")
    fi
    
    # Clean up
    rm -f "$QUERY_RESULT_FILE"
    
    echo "$result"
}

get_current_status() {
    timeout 2 ign topic -e -t /gnb/heatmap/status -n 1 2>/dev/null | \
        grep -oP 'data: "\K[^"]+' || echo "unavailable"
}

# =============================================================================
# Data Collection Functions
# =============================================================================

collect_scenario_data() {
    local scenario_key=$1
    local scenario_name=$2
    local output_file="${RESULTS_DIR}/raw_data/${scenario_key}.csv"
    
    log_info "Collecting data for scenario: $scenario_name"
    
    # CSV header
    echo "model,model_name,point_desc,x,y,z,signal_dbm,distance_m" > "$output_file"
    
    for model_entry in "${MODELS[@]}"; do
        local model_key="${model_entry%%:*}"
        local model_name="${model_entry#*:}"
        
        switch_model "$model_key"
        
        for point_entry in "${TEST_POINTS[@]}"; do
            local coords="${point_entry%%:*}"
            local point_desc="${point_entry#*:}"
            
            IFS=',' read -r x y z <<< "$coords"
            
            local signal=$(query_signal "$x" "$y" "$z")
            local distance=$(echo "scale=2; sqrt($x*$x + $y*$y)" | bc)
            
            echo "${model_key},\"${model_name}\",\"${point_desc}\",${x},${y},${z},${signal},${distance}" >> "$output_file"
            
            # Progress indicator
            printf "."
        done
        echo ""
    done
    
    log_success "Data saved to: $output_file"
}

# =============================================================================
# Analysis Functions
# =============================================================================

calculate_statistics() {
    local csv_file=$1
    local output_file=$2
    
    log_info "Calculating statistics from: $csv_file"
    
    # Use awk for statistics calculation
    awk -F',' '
    BEGIN {
        OFS=","
    }
    NR > 1 {
        model = $1
        signal = $7
        
        if (signal > -900) {  # Valid reading
            count[model]++
            sum[model] += signal
            sumsq[model] += signal * signal
            
            if (!(model in min) || signal < min[model]) min[model] = signal
            if (!(model in max) || signal > max[model]) max[model] = signal
            
            # Coverage counting
            if (signal >= -70) excellent[model]++
            else if (signal >= -85) good[model]++
            else if (signal >= -100) fair[model]++
            else if (signal >= -115) poor[model]++
            else noservice[model]++
        }
    }
    END {
        print "model,count,mean_dbm,std_dbm,min_dbm,max_dbm,excellent_pct,good_pct,fair_pct,poor_pct,no_service_pct"
        for (m in count) {
            n = count[m]
            mean = sum[m] / n
            variance = (sumsq[m] / n) - (mean * mean)
            std = sqrt(variance > 0 ? variance : 0)
            
            exc_pct = (excellent[m] / n) * 100
            good_pct = (good[m] / n) * 100
            fair_pct = (fair[m] / n) * 100
            poor_pct = (poor[m] / n) * 100
            nos_pct = (noservice[m] / n) * 100
            
            printf "%s,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\n", \
                m, n, mean, std, min[m], max[m], exc_pct, good_pct, fair_pct, poor_pct, nos_pct
        }
    }
    ' "$csv_file" > "$output_file"
    
    log_success "Statistics saved to: $output_file"
}

# =============================================================================
# Report Generation
# =============================================================================

generate_summary_report() {
    local report_file="${RESULTS_DIR}/comparison_report.txt"
    local csv_summary="${RESULTS_DIR}/summary_all_scenarios.csv"
    
    log_info "Generating summary report..."
    
    cat > "$report_file" << 'EOF'
================================================================================
        RF PROPAGATION MODEL COMPARISON REPORT
        Generated for Conference Paper Analysis
================================================================================

EOF
    
    echo "Report generated: $(date)" >> "$report_file"
    echo "Output directory: ${RESULTS_DIR}" >> "$report_file"
    echo "" >> "$report_file"
    
    # Combined CSV header
    echo "scenario,model,count,mean_dbm,std_dbm,min_dbm,max_dbm,excellent_pct,good_pct,fair_pct,poor_pct,no_service_pct" > "$csv_summary"
    
    for scenario_entry in "${SCENARIOS[@]}"; do
        local scenario_key="${scenario_entry%%:*}"
        local scenario_name="${scenario_entry#*:}"
        local stats_file="${RESULTS_DIR}/raw_data/${scenario_key}_stats.csv"
        
        if [ -f "$stats_file" ]; then
            echo "--------------------------------------------------------------------------------" >> "$report_file"
            echo "SCENARIO: ${scenario_name}" >> "$report_file"
            echo "--------------------------------------------------------------------------------" >> "$report_file"
            echo "" >> "$report_file"
            
            # Format stats as table
            echo "Model                      | Mean(dBm) | Std   | Min    | Max    | Coverage Quality Distribution" >> "$report_file"
            echo "---------------------------|-----------|-------|--------|--------|--------------------------------" >> "$report_file"
            
            tail -n +2 "$stats_file" | while IFS=',' read -r model count mean std min max exc good fair poor nos; do
                # Get full model name
                local model_name=""
                for m in "${MODELS[@]}"; do
                    if [[ "${m%%:*}" == "$model" ]]; then
                        model_name="${m#*:}"
                        break
                    fi
                done
                
                printf "%-26s | %9s | %5s | %6s | %6s | E:%.0f%% G:%.0f%% F:%.0f%% P:%.0f%% N:%.0f%%\n" \
                    "$model_name" "$mean" "$std" "$min" "$max" "$exc" "$good" "$fair" "$poor" "$nos" >> "$report_file"
                
                # Add to combined CSV
                echo "${scenario_key},${model},${count},${mean},${std},${min},${max},${exc},${good},${fair},${poor},${nos}" >> "$csv_summary"
            done
            
            echo "" >> "$report_file"
        fi
    done
    
    # Add interpretation guide
    cat >> "$report_file" << 'EOF'

================================================================================
INTERPRETATION GUIDE
================================================================================

Signal Quality Thresholds:
  Excellent (E): >= -70 dBm   - High throughput, reliable connection
  Good (G):      >= -85 dBm   - Good performance for most applications
  Fair (F):      >= -100 dBm  - Usable but may have reduced throughput
  Poor (P):      >= -115 dBm  - Marginal coverage, possible dropouts
  No Service (N): < -115 dBm  - Below sensitivity threshold

Model Characteristics:
  - Free Space Path Loss: Theoretical baseline, no obstacles
  - 3GPP UMi: Optimized for street-level small cells (< 10m height)
  - 3GPP UMa: Optimized for elevated macro cells (25-35m height)
  - Ray Tracing: Accounts for reflections, computationally intensive
  - Hybrid: Combines 3GPP statistical model with ray tracing multipath

Scenario Selection Guidelines:
  - Open Field: Use FSPL or 3GPP models (should converge)
  - Dense Urban: Hybrid or Ray Tracing recommended
  - Street Canyon: 3GPP UMi optimal
  - Elevated Macro: 3GPP UMa optimal
  - Indoor-Outdoor: Hybrid recommended for wall penetration

================================================================================
EOF
    
    log_success "Report saved to: $report_file"
    log_success "Combined CSV saved to: $csv_summary"
}

generate_latex_table() {
    local latex_file="${RESULTS_DIR}/latex_tables.tex"
    
    log_info "Generating LaTeX tables..."
    
    cat > "$latex_file" << 'EOF'
% LaTeX tables for conference paper
% Auto-generated by run_comparison.sh

\begin{table*}[htbp]
\centering
\caption{Propagation Model Comparison Across Scenarios}
\label{tab:model_comparison}
\begin{tabular}{llrrrrrrrrr}
\toprule
\textbf{Scenario} & \textbf{Model} & \textbf{Mean} & \textbf{Std} & \textbf{Min} & \textbf{Max} & \multicolumn{5}{c}{\textbf{Coverage (\%)}} \\
 & & \textbf{(dBm)} & \textbf{(dB)} & \textbf{(dBm)} & \textbf{(dBm)} & \textbf{Exc} & \textbf{Good} & \textbf{Fair} & \textbf{Poor} & \textbf{None} \\
\midrule
EOF
    
    local csv_summary="${RESULTS_DIR}/summary_all_scenarios.csv"
    
    if [ -f "$csv_summary" ]; then
        local current_scenario=""
        
        tail -n +2 "$csv_summary" | while IFS=',' read -r scenario model count mean std min max exc good fair poor nos; do
            # Get display names
            local scenario_name=""
            for s in "${SCENARIOS[@]}"; do
                if [[ "${s%%:*}" == "$scenario" ]]; then
                    scenario_name="${s#*:}"
                    break
                fi
            done
            
            local model_name=""
            for m in "${MODELS[@]}"; do
                if [[ "${m%%:*}" == "$model" ]]; then
                    model_name="${m#*:}"
                    break
                fi
            done
            
            # Add midrule between scenarios
            if [[ "$scenario" != "$current_scenario" && ! -z "$current_scenario" ]]; then
                echo "\\midrule" >> "$latex_file"
            fi
            current_scenario="$scenario"
            
            printf "%s & %s & %.1f & %.1f & %.1f & %.1f & %.0f & %.0f & %.0f & %.0f & %.0f \\\\\\\\\n" \
                "$scenario_name" "$model_name" "$mean" "$std" "$min" "$max" \
                "$exc" "$good" "$fair" "$poor" "$nos" >> "$latex_file"
        done
    fi
    
    cat >> "$latex_file" << 'EOF'
\bottomrule
\end{tabular}
\end{table*}
EOF
    
    log_success "LaTeX tables saved to: $latex_file"
}

# =============================================================================
# Main Execution
# =============================================================================

run_single_scenario() {
    local scenario_key=$1
    local scenario_name=$2
    
    log_info "=========================================="
    log_info "Running scenario: $scenario_name"
    log_info "=========================================="
    
    if start_gazebo "$scenario_key"; then
        collect_scenario_data "$scenario_key" "$scenario_name"
        
        # Calculate statistics
        local raw_file="${RESULTS_DIR}/raw_data/${scenario_key}.csv"
        local stats_file="${RESULTS_DIR}/raw_data/${scenario_key}_stats.csv"
        calculate_statistics "$raw_file" "$stats_file"
        
        stop_gazebo
        return 0
    else
        log_error "Failed to run scenario: $scenario_name"
        stop_gazebo
        return 1
    fi
}

run_all_scenarios() {
    local failed=0
    
    for scenario_entry in "${SCENARIOS[@]}"; do
        local scenario_key="${scenario_entry%%:*}"
        local scenario_name="${scenario_entry#*:}"
        
        if ! run_single_scenario "$scenario_key" "$scenario_name"; then
            ((failed++))
        fi
        
        # Brief pause between scenarios
        sleep 2
    done
    
    return $failed
}

print_usage() {
    cat << EOF
Usage: $0 [OPTIONS] [COMMAND]

Commands:
  all                 Run all scenarios (default)
  scenario <name>     Run single scenario
  report              Generate report from existing data
  list                List available scenarios and models

Options:
  -h, --help          Show this help message
  -o, --output DIR    Set output directory
  -q, --quick         Quick mode (fewer test points)
  
Examples:
  $0                          # Run all scenarios
  $0 scenario open_field      # Run single scenario
  $0 report                   # Generate report from existing data
  
Available Scenarios:
EOF
    for s in "${SCENARIOS[@]}"; do
        echo "  - ${s%%:*}: ${s#*:}"
    done
    
    echo ""
    echo "Available Models:"
    for m in "${MODELS[@]}"; do
        echo "  - ${m%%:*}: ${m#*:}"
    done
}

# Cleanup on exit
cleanup() {
    stop_gazebo
    # Clean up temp file if it exists
    rm -f "$QUERY_RESULT_FILE"
}
trap cleanup EXIT

# Main
main() {
    echo ""
    echo "========================================================"
    echo "  RF Propagation Model Comparison Tool"
    echo "  For 5G Simulation Research"
    echo "========================================================"
    echo ""
    
    check_dependencies
    setup_output_dir
    
    local command="${1:-all}"
    
    case "$command" in
        all)
            run_all_scenarios
            generate_summary_report
            generate_latex_table
            ;;
        scenario)
            if [ -z "$2" ]; then
                log_error "Please specify scenario name"
                print_usage
                exit 1
            fi
            local found=0
            for s in "${SCENARIOS[@]}"; do
                if [[ "${s%%:*}" == *"$2"* ]]; then
                    run_single_scenario "${s%%:*}" "${s#*:}"
                    found=1
                    break
                fi
            done
            if [ $found -eq 0 ]; then
                log_error "Scenario not found: $2"
                exit 1
            fi
            ;;
        report)
            generate_summary_report
            generate_latex_table
            ;;
        list)
            print_usage
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
    log_success "Comparison complete!"
    log_info "Results saved to: ${RESULTS_DIR}"
    echo ""
}

main "$@"
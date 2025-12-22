#!/usr/bin/env python3
"""
Multi-gNB Deployment KPI Comparison Plots
==========================================
Generates publication-ready visualizations comparing deployment configurations
across multiple Key Performance Indicators (KPIs).

Usage:
    python generate_kpi_comparison_plots.py <results_directory>
    python generate_kpi_comparison_plots.py multi_gnb_results/20241215_120000
"""

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from scipy import stats
import warnings
warnings.filterwarnings('ignore')

# =============================================================================
# Configuration
# =============================================================================

# Publication-ready settings
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.titlesize': 12,
    'axes.labelsize': 11,
    'xtick.labelsize': 9,
    'ytick.labelsize': 10,
    'legend.fontsize': 9,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.axisbelow': True
})

# Deployment configurations (matching run_multi_gnb_comparison.sh)
DEPLOYMENT_ORDER = [
    'single_central',
    'single_elevated', 
    'dual_symmetric',
    'dual_diagonal',
    'dual_heterogeneous',
    'tri_triangle',
    'tri_linear',
    'tri_heterogeneous',
    'penta_cross',
    'penta_pentagon'
]

DEPLOYMENT_LABELS = {
    'single_central': 'Single\nCentral',
    'single_elevated': 'Single\nElevated',
    'dual_symmetric': 'Dual\nSymmetric',
    'dual_diagonal': 'Dual\nDiagonal',
    'dual_heterogeneous': 'Dual\nHetero',
    'tri_triangle': 'Triangle',
    'tri_linear': 'Linear\nArray',
    'tri_heterogeneous': 'Tri-\nHetero',
    'penta_cross': 'Cross\nPattern',
    'penta_pentagon': 'Pentagon'
}

# Short labels for tight spaces
DEPLOYMENT_LABELS_SHORT = {
    'single_central': 'S-Cen',
    'single_elevated': 'S-Elev',
    'dual_symmetric': 'D-Sym',
    'dual_diagonal': 'D-Diag',
    'dual_heterogeneous': 'D-Het',
    'tri_triangle': 'Tri',
    'tri_linear': 'Lin',
    'tri_heterogeneous': 'T-Het',
    'penta_cross': 'Cross',
    'penta_pentagon': 'Pent'
}

# Color palette for deployments (colorblind-friendly)
DEPLOYMENT_COLORS = {
    'single_central': '#1f77b4',
    'single_elevated': '#17becf',
    'dual_symmetric': '#2ca02c',
    'dual_diagonal': '#98df8a',
    'dual_heterogeneous': '#ff7f0e',
    'tri_triangle': '#d62728',
    'tri_linear': '#ff9896',
    'tri_heterogeneous': '#9467bd',
    'penta_cross': '#8c564b',
    'penta_pentagon': '#e377c2'
}

# Category colors for grouped charts
CATEGORY_COLORS = {
    'single': '#3498db',
    'dual': '#2ecc71', 
    'triple': '#e74c3c',
    'penta': '#9b59b6'
}

# Thresholds
COVERAGE_THRESHOLD_DBM = -100  # Good coverage
HOLE_THRESHOLD_DBM = -115      # No coverage
SINR_GOOD_THRESHOLD_DB = 10    # Good SINR
SINR_INTERFERENCE_THRESHOLD_DB = 0  # Interference-limited
HANDOVER_MARGIN_DB = 6         # Handover region margin


# =============================================================================
# Data Loading Functions
# =============================================================================

def load_grid_data(filepath):
    """Load grid data from CSV file."""
    try:
        df = pd.read_csv(filepath)
        return df
    except Exception as e:
        print(f"  Error loading {filepath}: {e}")
        return None


def find_deployment_files(results_dir, deployment_key, model='3gpp_umi', mode='best_server'):
    """Find the data file for a specific deployment configuration."""
    raw_dir = os.path.join(results_dir, 'raw_data')
    
    # Try different naming patterns
    patterns = [
        f"*_{deployment_key}_{model}_{mode}.csv",
        f"*{deployment_key}*{model}*{mode}*.csv",
        f"{deployment_key}*.csv"
    ]
    
    for pattern in patterns:
        matches = glob.glob(os.path.join(raw_dir, pattern))
        # Filter out stats files
        matches = [m for m in matches if '_stats' not in m]
        if matches:
            return matches[0]
    
    return None


def load_all_deployments(results_dir, model='3gpp_umi', mode='best_server'):
    """Load data for all deployment configurations."""
    data = {}
    
    for dep in DEPLOYMENT_ORDER:
        filepath = find_deployment_files(results_dir, dep, model, mode)
        if filepath:
            df = load_grid_data(filepath)
            if df is not None and len(df) > 0:
                data[dep] = df
                print(f"  Loaded {dep}: {len(df)} points")
            else:
                print(f"  Warning: Empty data for {dep}")
        else:
            print(f"  Warning: No file found for {dep}")
    
    return data


def load_sinr_data(results_dir, model='3gpp_umi'):
    """Load SINR-specific data for each deployment."""
    return load_all_deployments(results_dir, model, 'sinr')


# =============================================================================
# KPI Calculation Functions
# =============================================================================

def calculate_all_kpis(data_dict, sinr_data_dict=None):
    """Calculate all KPIs for each deployment configuration."""
    kpis = {dep: {} for dep in DEPLOYMENT_ORDER}
    
    for dep in DEPLOYMENT_ORDER:
        if dep not in data_dict:
            continue
            
        df = data_dict[dep]
        total_points = len(df)
        
        if total_points == 0:
            continue
        
        # Signal column name (handle different naming conventions)
        signal_col = 'best_signal_dbm' if 'best_signal_dbm' in df.columns else 'best_signal'
        sinr_col = 'sinr_db' if 'sinr_db' in df.columns else 'sinr'
        gnb_col = 'best_gnb_id' if 'best_gnb_id' in df.columns else 'best_gnb'
        
        if signal_col not in df.columns:
            print(f"  Warning: No signal column found for {dep}")
            continue
        
        signals = df[signal_col].values
        
        # 1. Coverage Probability (% of area with signal >= threshold)
        covered = np.sum(signals >= COVERAGE_THRESHOLD_DBM)
        kpis[dep]['coverage_probability'] = (covered / total_points) * 100
        
        # 2. Coverage Hole Ratio (% of area with no coverage)
        holes = np.sum(signals < HOLE_THRESHOLD_DBM)
        kpis[dep]['coverage_hole_ratio'] = (holes / total_points) * 100
        
        # Get SINR data
        if sinr_data_dict and dep in sinr_data_dict:
            sinr_df = sinr_data_dict[dep]
            if sinr_col in sinr_df.columns:
                sinr_values = sinr_df[sinr_col].values
            elif 'sinr_db' in df.columns:
                sinr_values = df['sinr_db'].values
            else:
                sinr_values = df[sinr_col].values if sinr_col in df.columns else None
        elif sinr_col in df.columns:
            sinr_values = df[sinr_col].values
        else:
            sinr_values = None
        
        if sinr_values is not None and len(sinr_values) > 0:
            # Filter out invalid values
            valid_sinr = sinr_values[~np.isnan(sinr_values) & (sinr_values > -100)]
            
            if len(valid_sinr) > 0:
                # 3. Mean SINR
                kpis[dep]['mean_sinr'] = np.mean(valid_sinr)
                
                # 5. Interference-Limited Fraction (SINR < threshold)
                interference_limited = np.sum(valid_sinr < SINR_INTERFERENCE_THRESHOLD_DB)
                kpis[dep]['interference_limited_fraction'] = (interference_limited / len(valid_sinr)) * 100
                
                # 9. 5th-Percentile SINR (for box plots)
                kpis[dep]['sinr_5th_percentile'] = np.percentile(valid_sinr, 5)
                kpis[dep]['sinr_distribution'] = valid_sinr
        
        # Calculate handover metrics (for multi-gNB deployments)
        if gnb_col in df.columns:
            gnb_ids = df[gnb_col].values
            unique_gnbs = np.unique(gnb_ids[gnb_ids >= 0])
            
            if len(unique_gnbs) > 1:
                # Need per-gNB signal data to calculate handover regions
                # For now, estimate based on signal variance in local areas
                kpis[dep]['num_gnbs'] = len(unique_gnbs)
                
                # Estimate handover region as areas where multiple gNBs serve nearby
                # This is simplified - in real data we'd have all gNB signals
                kpis[dep]['handover_region_pct'] = estimate_handover_region(df, gnb_col)
            else:
                kpis[dep]['num_gnbs'] = 1
                kpis[dep]['handover_region_pct'] = 0
        
    # Calculate derived KPIs
    baseline = 'single_central'
    if baseline in kpis and 'mean_sinr' in kpis.get(baseline, {}):
        baseline_sinr = kpis[baseline].get('mean_sinr', 0)
        baseline_coverage = kpis[baseline].get('coverage_probability', 0)
        
        for dep in DEPLOYMENT_ORDER:
            if dep in kpis:
                # 4. SINR Degradation (compared to single gNB baseline)
                if 'mean_sinr' in kpis[dep]:
                    kpis[dep]['sinr_degradation'] = baseline_sinr - kpis[dep]['mean_sinr']
                
                # 8. Coverage Gain (improvement over baseline)
                if 'coverage_probability' in kpis[dep]:
                    kpis[dep]['coverage_gain'] = kpis[dep]['coverage_probability'] - baseline_coverage
                
                # 7. Boundary SINR (mean SINR in handover regions)
                # Simplified: use lower percentile SINR as proxy
                if 'sinr_distribution' in kpis[dep]:
                    kpis[dep]['boundary_sinr'] = np.percentile(kpis[dep]['sinr_distribution'], 25)
    
    return kpis


def estimate_handover_region(df, gnb_col):
    """Estimate handover region percentage based on gNB boundaries."""
    # Group by spatial grid and find transition points
    gnb_ids = df[gnb_col].values
    x_vals = df['x'].values
    y_vals = df['y'].values
    
    handover_count = 0
    total_count = len(df)
    
    # Check each point's neighbors
    for i in range(len(df)):
        x, y = x_vals[i], y_vals[i]
        gnb = gnb_ids[i]
        
        # Find nearby points
        distances = np.sqrt((x_vals - x)**2 + (y_vals - y)**2)
        nearby_mask = (distances > 0) & (distances < 10)  # Within 10m
        
        if np.any(nearby_mask):
            nearby_gnbs = gnb_ids[nearby_mask]
            # If any neighbor has different serving gNB, this is handover region
            if np.any(nearby_gnbs != gnb):
                handover_count += 1
    
    return (handover_count / total_count) * 100 if total_count > 0 else 0


# =============================================================================
# Plotting Functions
# =============================================================================

def plot_kpi_bars(kpis, kpi_name, ylabel, title, output_path, 
                  threshold_line=None, threshold_label=None,
                  invert=False, show_values=True):
    """Create a bar chart for a single KPI across all deployments."""
    fig, ax = plt.subplots(figsize=(12, 5))
    
    x_positions = np.arange(len(DEPLOYMENT_ORDER))
    values = []
    colors = []
    
    for dep in DEPLOYMENT_ORDER:
        if dep in kpis and kpi_name in kpis[dep]:
            values.append(kpis[dep][kpi_name])
        else:
            values.append(0)
        colors.append(DEPLOYMENT_COLORS.get(dep, '#7f7f7f'))
    
    bars = ax.bar(x_positions, values, color=colors, edgecolor='black', linewidth=0.5)
    
    # Add threshold line if specified
    if threshold_line is not None:
        ax.axhline(y=threshold_line, color='red', linestyle='--', linewidth=1.5, 
                   label=threshold_label if threshold_label else f'Threshold: {threshold_line}')
        ax.legend(loc='upper right')
    
    # Add value labels on bars
    if show_values:
        for bar, val in zip(bars, values):
            height = bar.get_height()
            ax.annotate(f'{val:.1f}',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3),
                       textcoords="offset points",
                       ha='center', va='bottom', fontsize=8)
    
    ax.set_xlabel('Deployment Configuration')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xticks(x_positions)
    ax.set_xticklabels([DEPLOYMENT_LABELS_SHORT.get(d, d) for d in DEPLOYMENT_ORDER], 
                       rotation=45, ha='right')
    
    if invert:
        ax.invert_yaxis()
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_all_kpis_grid(kpis, output_path):
    """Create a grid of all 7 main KPIs in a single figure."""
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    axes = axes.flatten()
    
    kpi_configs = [
        ('coverage_probability', 'Coverage Probability (%)', 'Coverage Probability', None),
        ('coverage_hole_ratio', 'Coverage Hole Ratio (%)', 'Coverage Hole Ratio', None),
        ('mean_sinr', 'Mean SINR (dB)', 'Mean SINR', SINR_GOOD_THRESHOLD_DB),
        ('sinr_degradation', 'SINR Degradation (dB)', 'SINR Degradation vs Single Central', 0),
        ('interference_limited_fraction', 'Interference-Limited (%)', 'Interference-Limited Fraction', None),
        ('handover_region_pct', 'Handover Region (%)', 'Handover Region Percentage', None),
        ('boundary_sinr', 'Boundary SINR (dB)', 'Boundary SINR (25th percentile)', SINR_INTERFERENCE_THRESHOLD_DB),
    ]
    
    x_positions = np.arange(len(DEPLOYMENT_ORDER))
    
    for idx, (kpi_name, ylabel, title, threshold) in enumerate(kpi_configs):
        ax = axes[idx]
        
        values = []
        colors = []
        for dep in DEPLOYMENT_ORDER:
            if dep in kpis and kpi_name in kpis[dep]:
                values.append(kpis[dep][kpi_name])
            else:
                values.append(np.nan)
            colors.append(DEPLOYMENT_COLORS.get(dep, '#7f7f7f'))
        
        bars = ax.bar(x_positions, values, color=colors, edgecolor='black', linewidth=0.5)
        
        if threshold is not None:
            ax.axhline(y=threshold, color='red', linestyle='--', linewidth=1, alpha=0.7)
        
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.set_xticks(x_positions)
        ax.set_xticklabels([DEPLOYMENT_LABELS_SHORT.get(d, d) for d in DEPLOYMENT_ORDER], 
                          rotation=45, ha='right', fontsize=8)
    
    # Hide unused subplots
    for idx in range(len(kpi_configs), len(axes)):
        axes[idx].set_visible(False)
    
    # Add legend
    legend_elements = [mpatches.Patch(facecolor=DEPLOYMENT_COLORS[dep], 
                                       edgecolor='black', label=DEPLOYMENT_LABELS_SHORT[dep])
                      for dep in DEPLOYMENT_ORDER]
    fig.legend(handles=legend_elements, loc='lower right', ncol=5, fontsize=8,
              bbox_to_anchor=(0.98, 0.02))
    
    plt.tight_layout(rect=[0, 0.08, 1, 1])
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_coverage_gain_waterfall(kpis, output_path):
    """Create a waterfall chart showing incremental coverage gains."""
    fig, ax = plt.subplots(figsize=(14, 6))
    
    # Sort deployments by number of gNBs and coverage
    deployment_info = []
    for dep in DEPLOYMENT_ORDER:
        if dep in kpis and 'coverage_probability' in kpis[dep]:
            num_gnbs = kpis[dep].get('num_gnbs', 1)
            coverage = kpis[dep]['coverage_probability']
            gain = kpis[dep].get('coverage_gain', 0)
            deployment_info.append((dep, num_gnbs, coverage, gain))
    
    # Sort by coverage
    deployment_info.sort(key=lambda x: x[2])
    
    x_positions = np.arange(len(deployment_info))
    
    # Create waterfall effect
    baseline_coverage = deployment_info[0][2] if deployment_info else 0
    
    # Plot bars
    coverages = [d[2] for d in deployment_info]
    gains = [d[3] for d in deployment_info]
    labels = [DEPLOYMENT_LABELS_SHORT.get(d[0], d[0]) for d in deployment_info]
    colors = [DEPLOYMENT_COLORS.get(d[0], '#7f7f7f') for d in deployment_info]
    
    bars = ax.bar(x_positions, coverages, color=colors, edgecolor='black', linewidth=0.5)
    
    # Add gain annotations
    for i, (bar, gain, coverage) in enumerate(zip(bars, gains, coverages)):
        # Coverage value
        ax.annotate(f'{coverage:.1f}%',
                   xy=(bar.get_x() + bar.get_width() / 2, bar.get_height()),
                   xytext=(0, 3), textcoords="offset points",
                   ha='center', va='bottom', fontsize=9, fontweight='bold')
        
        # Gain indicator (if not baseline)
        if i > 0 and gain != 0:
            color = 'green' if gain > 0 else 'red'
            sign = '+' if gain > 0 else ''
            ax.annotate(f'{sign}{gain:.1f}%',
                       xy=(bar.get_x() + bar.get_width() / 2, bar.get_height() / 2),
                       ha='center', va='center', fontsize=8, color=color,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
    
    ax.set_xlabel('Deployment Configuration (sorted by coverage)')
    ax.set_ylabel('Coverage Probability (%)')
    ax.set_title('Coverage Probability Comparison with Gains vs Baseline')
    ax.set_xticks(x_positions)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.set_ylim(0, 105)
    
    # Add baseline reference line
    ax.axhline(y=baseline_coverage, color='blue', linestyle=':', linewidth=1.5,
               label=f'Single Central Baseline ({baseline_coverage:.1f}%)')
    ax.legend(loc='lower right')
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_sinr_distribution_boxplots(kpis, output_path):
    """Create box plots showing SINR distribution for each deployment."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Prepare data for box plots
    box_data = []
    labels = []
    colors = []
    percentile_5th = []
    
    for dep in DEPLOYMENT_ORDER:
        if dep in kpis and 'sinr_distribution' in kpis[dep]:
            sinr_dist = kpis[dep]['sinr_distribution']
            if len(sinr_dist) > 0:
                box_data.append(sinr_dist)
                labels.append(DEPLOYMENT_LABELS_SHORT.get(dep, dep))
                colors.append(DEPLOYMENT_COLORS.get(dep, '#7f7f7f'))
                percentile_5th.append(kpis[dep].get('sinr_5th_percentile', np.nan))
    
    if not box_data:
        print("  No SINR distribution data available for box plots")
        plt.close()
        return
    
    # Left plot: Box plots
    ax1 = axes[0]
    bp = ax1.boxplot(box_data, patch_artist=True, labels=labels)
    
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax1.axhline(y=SINR_GOOD_THRESHOLD_DB, color='green', linestyle='--', 
                label=f'Good SINR ({SINR_GOOD_THRESHOLD_DB} dB)')
    ax1.axhline(y=SINR_INTERFERENCE_THRESHOLD_DB, color='red', linestyle='--',
                label=f'Interference Threshold ({SINR_INTERFERENCE_THRESHOLD_DB} dB)')
    
    ax1.set_xlabel('Deployment Configuration')
    ax1.set_ylabel('SINR (dB)')
    ax1.set_title('SINR Distribution by Deployment')
    ax1.tick_params(axis='x', rotation=45)
    ax1.legend(loc='upper right', fontsize=8)
    
    # Right plot: 5th percentile SINR comparison
    ax2 = axes[1]
    x_pos = np.arange(len(labels))
    bars = ax2.bar(x_pos, percentile_5th, color=colors, edgecolor='black', linewidth=0.5)
    
    # Add value labels
    for bar, val in zip(bars, percentile_5th):
        if not np.isnan(val):
            ax2.annotate(f'{val:.1f}',
                        xy=(bar.get_x() + bar.get_width() / 2, bar.get_height()),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=8)
    
    ax2.axhline(y=SINR_INTERFERENCE_THRESHOLD_DB, color='red', linestyle='--',
                label=f'Interference Threshold ({SINR_INTERFERENCE_THRESHOLD_DB} dB)')
    
    ax2.set_xlabel('Deployment Configuration')
    ax2.set_ylabel('5th Percentile SINR (dB)')
    ax2.set_title('5th Percentile SINR (Cell-Edge Performance)')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(labels, rotation=45, ha='right')
    ax2.legend(loc='lower right', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_coverage_vs_interference_tradeoff(kpis, output_path):
    """Plot coverage probability vs interference-limited fraction trade-off."""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    for dep in DEPLOYMENT_ORDER:
        if dep not in kpis:
            continue
        
        coverage = kpis[dep].get('coverage_probability', None)
        interference = kpis[dep].get('interference_limited_fraction', None)
        
        if coverage is not None and interference is not None:
            ax.scatter(interference, coverage, 
                      c=[DEPLOYMENT_COLORS.get(dep, '#7f7f7f')],
                      s=200, edgecolors='black', linewidth=1.5,
                      label=DEPLOYMENT_LABELS_SHORT.get(dep, dep), zorder=5)
            
            # Add label
            ax.annotate(DEPLOYMENT_LABELS_SHORT.get(dep, dep),
                       xy=(interference, coverage),
                       xytext=(5, 5), textcoords='offset points',
                       fontsize=8, fontweight='bold')
    
    ax.set_xlabel('Interference-Limited Fraction (%)')
    ax.set_ylabel('Coverage Probability (%)')
    ax.set_title('Coverage vs Interference Trade-off')
    
    # Add quadrant labels
    ax.axhline(y=90, color='green', linestyle=':', alpha=0.5)
    ax.axvline(x=10, color='red', linestyle=':', alpha=0.5)
    
    ax.text(5, 95, 'Ideal\n(High Coverage,\nLow Interference)', 
            ha='center', va='center', fontsize=9, color='green',
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
    
    ax.legend(loc='lower left', ncol=2, fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_grouped_kpi_comparison(kpis, output_path):
    """Create grouped bar chart comparing single/dual/triple/penta deployments."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Group deployments by category
    categories = {
        'Single': ['single_central', 'single_elevated'],
        'Dual': ['dual_symmetric', 'dual_diagonal', 'dual_heterogeneous'],
        'Triple': ['tri_triangle', 'tri_linear', 'tri_heterogeneous'],
        'Penta': ['penta_cross', 'penta_pentagon']
    }
    
    kpi_list = [
        ('coverage_probability', 'Coverage Probability (%)', axes[0, 0]),
        ('mean_sinr', 'Mean SINR (dB)', axes[0, 1]),
        ('interference_limited_fraction', 'Interference-Limited (%)', axes[1, 0]),
        ('handover_region_pct', 'Handover Region (%)', axes[1, 1])
    ]
    
    for kpi_name, ylabel, ax in kpi_list:
        x = 0
        xticks = []
        xticklabels = []
        
        for cat_name, deps in categories.items():
            for dep in deps:
                if dep in kpis and kpi_name in kpis[dep]:
                    val = kpis[dep][kpi_name]
                    ax.bar(x, val, color=DEPLOYMENT_COLORS.get(dep, '#7f7f7f'),
                          edgecolor='black', linewidth=0.5)
                    xticks.append(x)
                    xticklabels.append(DEPLOYMENT_LABELS_SHORT.get(dep, dep))
                    x += 1
            x += 0.5  # Gap between categories
        
        ax.set_ylabel(ylabel)
        ax.set_title(kpi_name.replace('_', ' ').title())
        ax.set_xticks(xticks)
        ax.set_xticklabels(xticklabels, rotation=45, ha='right', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def generate_kpi_summary_table(kpis, output_path):
    """Generate a summary table of all KPIs as CSV."""
    rows = []
    
    columns = [
        'Deployment',
        'Coverage Probability (%)',
        'Coverage Hole Ratio (%)',
        'Mean SINR (dB)',
        'SINR Degradation (dB)',
        'Interference-Limited (%)',
        'Handover Region (%)',
        'Boundary SINR (dB)',
        'Coverage Gain (%)',
        '5th Percentile SINR (dB)'
    ]
    
    for dep in DEPLOYMENT_ORDER:
        if dep not in kpis:
            continue
        
        k = kpis[dep]
        row = {
            'Deployment': DEPLOYMENT_LABELS_SHORT.get(dep, dep),
            'Coverage Probability (%)': k.get('coverage_probability', np.nan),
            'Coverage Hole Ratio (%)': k.get('coverage_hole_ratio', np.nan),
            'Mean SINR (dB)': k.get('mean_sinr', np.nan),
            'SINR Degradation (dB)': k.get('sinr_degradation', np.nan),
            'Interference-Limited (%)': k.get('interference_limited_fraction', np.nan),
            'Handover Region (%)': k.get('handover_region_pct', np.nan),
            'Boundary SINR (dB)': k.get('boundary_sinr', np.nan),
            'Coverage Gain (%)': k.get('coverage_gain', np.nan),
            '5th Percentile SINR (dB)': k.get('sinr_5th_percentile', np.nan)
        }
        rows.append(row)
    
    df = pd.DataFrame(rows)
    df.to_csv(output_path, index=False, float_format='%.2f')
    print(f"  Saved: {output_path}")
    
    return df


def generate_latex_table(kpis, output_path):
    """Generate LaTeX table for the paper."""
    latex_content = r"""% Auto-generated KPI comparison table
\begin{table*}[htbp]
\centering
\caption{Multi-gNB Deployment KPI Comparison}
\label{tab:kpi_comparison}
\begin{tabular}{l|cc|ccc|cc|cc}
\toprule
\textbf{Deployment} & \textbf{Cov.} & \textbf{Holes} & \textbf{Mean} & \textbf{SINR} & \textbf{Interf.} & \textbf{H/O} & \textbf{Bound.} & \textbf{Cov.} & \textbf{5th \%} \\
 & \textbf{Prob.} & \textbf{Ratio} & \textbf{SINR} & \textbf{Degr.} & \textbf{Limit.} & \textbf{Region} & \textbf{SINR} & \textbf{Gain} & \textbf{SINR} \\
 & (\%) & (\%) & (dB) & (dB) & (\%) & (\%) & (dB) & (\%) & (dB) \\
\midrule
"""
    
    for dep in DEPLOYMENT_ORDER:
        if dep not in kpis:
            continue
        
        k = kpis[dep]
        name = DEPLOYMENT_LABELS_SHORT.get(dep, dep).replace('\n', ' ')
        
        vals = [
            k.get('coverage_probability', np.nan),
            k.get('coverage_hole_ratio', np.nan),
            k.get('mean_sinr', np.nan),
            k.get('sinr_degradation', np.nan),
            k.get('interference_limited_fraction', np.nan),
            k.get('handover_region_pct', np.nan),
            k.get('boundary_sinr', np.nan),
            k.get('coverage_gain', np.nan),
            k.get('sinr_5th_percentile', np.nan)
        ]
        
        val_strs = [f'{v:.1f}' if not np.isnan(v) else '--' for v in vals]
        
        latex_content += f"{name} & {' & '.join(val_strs)} \\\\\n"
    
    latex_content += r"""\bottomrule
\end{tabular}
\end{table*}
"""
    
    with open(output_path, 'w') as f:
        f.write(latex_content)
    
    print(f"  Saved: {output_path}")


# =============================================================================
# Main Function
# =============================================================================

def main(results_dir):
    """Main analysis function."""
    print(f"\n{'='*60}")
    print("  Multi-gNB KPI Comparison Analysis")
    print(f"{'='*60}\n")
    print(f"Results directory: {results_dir}\n")
    
    # Setup output directory
    figures_dir = os.path.join(results_dir, 'figures')
    os.makedirs(figures_dir, exist_ok=True)
    
    # Load data
    print("Loading deployment data...")
    data = load_all_deployments(results_dir)
    
    if not data:
        print("ERROR: No deployment data found!")
        print("Looking for alternative data sources...")
        
        # Try loading any CSV files
        raw_dir = os.path.join(results_dir, 'raw_data')
        if os.path.exists(raw_dir):
            all_csvs = glob.glob(os.path.join(raw_dir, '*.csv'))
            print(f"Found {len(all_csvs)} CSV files in raw_data/")
            for csv in all_csvs[:5]:
                print(f"  - {os.path.basename(csv)}")
        return
    
    print(f"\nLoaded {len(data)} deployment configurations\n")
    
    # Load SINR-specific data if available
    print("Loading SINR data...")
    sinr_data = load_sinr_data(results_dir)
    
    # Calculate all KPIs
    print("\nCalculating KPIs...")
    kpis = calculate_all_kpis(data, sinr_data)
    
    # Generate individual KPI plots
    print("\nGenerating individual KPI plots...")
    
    plot_kpi_bars(kpis, 'coverage_probability', 'Coverage Probability (%)',
                  'Coverage Probability by Deployment',
                  os.path.join(figures_dir, 'kpi_coverage_probability.pdf'),
                  threshold_line=90, threshold_label='Target: 90%')
    
    plot_kpi_bars(kpis, 'coverage_hole_ratio', 'Coverage Hole Ratio (%)',
                  'Coverage Hole Ratio by Deployment',
                  os.path.join(figures_dir, 'kpi_coverage_holes.pdf'),
                  threshold_line=5, threshold_label='Target: <5%')
    
    plot_kpi_bars(kpis, 'mean_sinr', 'Mean SINR (dB)',
                  'Mean SINR by Deployment',
                  os.path.join(figures_dir, 'kpi_mean_sinr.pdf'),
                  threshold_line=SINR_GOOD_THRESHOLD_DB, threshold_label='Good SINR')
    
    plot_kpi_bars(kpis, 'sinr_degradation', 'SINR Degradation (dB)',
                  'SINR Degradation vs Single Central Baseline',
                  os.path.join(figures_dir, 'kpi_sinr_degradation.pdf'),
                  threshold_line=0)
    
    plot_kpi_bars(kpis, 'interference_limited_fraction', 'Interference-Limited Fraction (%)',
                  'Interference-Limited Area Fraction',
                  os.path.join(figures_dir, 'kpi_interference_limited.pdf'),
                  threshold_line=10, threshold_label='Target: <10%')
    
    plot_kpi_bars(kpis, 'handover_region_pct', 'Handover Region (%)',
                  'Handover Region Percentage',
                  os.path.join(figures_dir, 'kpi_handover_region.pdf'))
    
    plot_kpi_bars(kpis, 'boundary_sinr', 'Boundary SINR (dB)',
                  'Boundary SINR (25th Percentile)',
                  os.path.join(figures_dir, 'kpi_boundary_sinr.pdf'),
                  threshold_line=SINR_INTERFERENCE_THRESHOLD_DB)
    
    # Generate composite plots
    print("\nGenerating composite plots...")
    
    plot_all_kpis_grid(kpis, os.path.join(figures_dir, 'kpi_all_grid.pdf'))
    
    plot_coverage_gain_waterfall(kpis, os.path.join(figures_dir, 'kpi_coverage_gain_waterfall.pdf'))
    
    plot_sinr_distribution_boxplots(kpis, os.path.join(figures_dir, 'kpi_sinr_distribution.pdf'))
    
    plot_coverage_vs_interference_tradeoff(kpis, os.path.join(figures_dir, 'kpi_coverage_vs_interference.pdf'))
    
    plot_grouped_kpi_comparison(kpis, os.path.join(figures_dir, 'kpi_grouped_comparison.pdf'))
    
    # Generate summary tables
    print("\nGenerating summary tables...")
    
    generate_kpi_summary_table(kpis, os.path.join(results_dir, 'kpi_summary.csv'))
    
    generate_latex_table(kpis, os.path.join(results_dir, 'kpi_table.tex'))
    
    print(f"\n{'='*60}")
    print("  Analysis Complete!")
    print(f"{'='*60}")
    print(f"\nFigures saved to: {figures_dir}")
    print(f"Summary saved to: {results_dir}\n")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python generate_kpi_comparison_plots.py <results_directory>")
        print("Example: python generate_kpi_comparison_plots.py multi_gnb_results/20241215_120000")
        sys.exit(1)
    
    results_dir = sys.argv[1]
    
    if not os.path.isdir(results_dir):
        print(f"Error: Directory not found: {results_dir}")
        sys.exit(1)
    
    main(results_dir)
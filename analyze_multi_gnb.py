#!/usr/bin/env python3
"""
Multi-gNB Deployment Analysis Script (FIXED)
=====================================
Generates publication-ready visualizations for the academic paper on
single vs multiple gNB deployment analysis.

Usage:
    python analyze_multi_gnb.py <results_directory>
    python analyze_multi_gnb.py multi_gnb_results/20241215_120000
"""

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from scipy import interpolate
from scipy.ndimage import gaussian_filter
import warnings
warnings.filterwarnings('ignore')

# Publication-ready font sizes
FONT_SIZES = {
    'title': 14,
    'label': 12,
    'tick': 10,
    'legend': 10,
    'annotation': 9
}

# Color schemes
GNB_COLORS = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', 
              '#ffff33', '#a65628', '#f781bf']
COVERAGE_CMAP = 'RdYlGn'
SINR_CMAP = 'RdYlBu'
SIGNAL_CMAP = 'jet'

plt.rcParams.update({
    'font.size': FONT_SIZES['tick'],
    'axes.titlesize': FONT_SIZES['title'],
    'axes.labelsize': FONT_SIZES['label'],
    'xtick.labelsize': FONT_SIZES['tick'],
    'ytick.labelsize': FONT_SIZES['tick'],
    'legend.fontsize': FONT_SIZES['legend'],
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1
})


def load_grid_data(filepath):
    """Load grid data from CSV file."""
    return pd.read_csv(filepath)


def load_all_results(results_dir):
    """Load all result files from a directory."""
    data = {}
    raw_dir = os.path.join(results_dir, 'raw_data')
    
    for csv_file in glob.glob(os.path.join(raw_dir, '*.csv')):
        basename = os.path.basename(csv_file).replace('.csv', '')
        if '_stats' not in basename:
            df = pd.read_csv(csv_file)
            # Debug: print data summary
            print(f"  Loaded {basename}: {len(df)} rows")
            if 'best_signal_dbm' in df.columns:
                print(f"    Signal range: {df['best_signal_dbm'].min():.1f} to {df['best_signal_dbm'].max():.1f} dBm")
            if 'best_gnb_id' in df.columns:
                print(f"    Unique gNB IDs: {sorted(df['best_gnb_id'].unique())}")
            data[basename] = df
    
    return data


def create_heatmap_grid(df, value_col, grid_size=None):
    """Convert DataFrame to 2D grid for heatmap plotting."""
    x_unique = np.sort(df['x'].unique())
    y_unique = np.sort(df['y'].unique())
    
    grid = np.full((len(y_unique), len(x_unique)), np.nan)
    
    x_idx = {v: i for i, v in enumerate(x_unique)}
    y_idx = {v: i for i, v in enumerate(y_unique)}
    
    for _, row in df.iterrows():
        xi = x_idx.get(row['x'])
        yi = y_idx.get(row['y'])
        if xi is not None and yi is not None:
            grid[yi, xi] = row[value_col]
    
    return grid, x_unique, y_unique


def plot_signal_heatmap(df, title, output_path, value_col='best_signal_dbm',
                        vmin=None, vmax=None, cmap=SIGNAL_CMAP, gnb_positions=None):
    """Plot signal strength heatmap."""
    fig, ax = plt.subplots(figsize=(8, 7))
    
    grid, x_vals, y_vals = create_heatmap_grid(df, value_col)
    
    # Check if we have valid data
    valid_data = grid[~np.isnan(grid)]
    if len(valid_data) == 0:
        print(f"  WARNING: No valid data for {title}")
        plt.close()
        return
    
    # Auto-calculate vmin/vmax if not provided, based on actual data
    if vmin is None:
        vmin = max(valid_data.min(), -120)  # Floor at -120 dBm
    if vmax is None:
        vmax = min(valid_data.max(), -30)   # Ceiling at -30 dBm
    
    # Ensure we have a valid range
    if vmax <= vmin:
        vmax = vmin + 10  # Ensure at least 10 dB range
    
    print(f"    Data range: {valid_data.min():.1f} to {valid_data.max():.1f}, plotting with vmin={vmin:.1f}, vmax={vmax:.1f}")
    
    # Apply slight smoothing for visual appeal (only if enough data)
    if grid.shape[0] > 3 and grid.shape[1] > 3:
        # Fill NaN with nearest neighbor before smoothing
        grid_filled = np.nan_to_num(grid, nan=vmin)
        grid_smooth = gaussian_filter(grid_filled, sigma=0.5)
    else:
        grid_smooth = np.nan_to_num(grid, nan=vmin)
    
    extent = [x_vals.min(), x_vals.max(), y_vals.min(), y_vals.max()]
    
    im = ax.imshow(grid_smooth, extent=extent, origin='lower',
                   cmap=cmap, vmin=vmin, vmax=vmax, aspect='equal')
    
    # Add gNB markers
    if gnb_positions is not None:
        for i, (gx, gy) in enumerate(gnb_positions):
            ax.plot(gx, gy, 'k^', markersize=12, markeredgewidth=2,
                   markerfacecolor=GNB_COLORS[i % len(GNB_COLORS)])
            ax.annotate(f'gNB{i}', (gx, gy), xytext=(5, 5),
                       textcoords='offset points', fontsize=FONT_SIZES['annotation'])
    
    cbar = plt.colorbar(im, ax=ax, label='Signal Strength (dBm)', shrink=0.8)
    
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_sinr_heatmap(df, title, output_path, gnb_positions=None):
    """Plot SINR heatmap with interference visualization."""
    fig, ax = plt.subplots(figsize=(8, 7))
    
    grid, x_vals, y_vals = create_heatmap_grid(df, 'sinr_db')
    
    # Check for valid data
    valid_data = grid[~np.isnan(grid)]
    if len(valid_data) == 0:
        print(f"  WARNING: No valid SINR data for {title}")
        plt.close()
        return
    
    # Auto-calculate range based on data
    data_min = valid_data.min()
    data_max = valid_data.max()
    vmin = max(data_min - 5, -20)
    vmax = min(data_max + 5, 40)
    
    print(f"    SINR range: {data_min:.1f} to {data_max:.1f} dB")
    
    # Fill NaN and smooth
    if grid.shape[0] > 3 and grid.shape[1] > 3:
        grid_filled = np.nan_to_num(grid, nan=0)
        grid_smooth = gaussian_filter(grid_filled, sigma=0.5)
    else:
        grid_smooth = np.nan_to_num(grid, nan=0)
    
    extent = [x_vals.min(), x_vals.max(), y_vals.min(), y_vals.max()]
    
    im = ax.imshow(grid_smooth, extent=extent, origin='lower',
                   cmap=SINR_CMAP, vmin=vmin, vmax=vmax, aspect='equal')
    
    # Overlay contours for SINR thresholds (only if we have variation)
    if data_max - data_min > 5:
        X, Y = np.meshgrid(x_vals, y_vals)
        levels = [l for l in [0, 5, 10, 20] if vmin < l < vmax]
        if levels:
            contours = ax.contour(X, Y, grid_smooth, levels=levels,
                                  colors='black', linewidths=0.5, linestyles='--')
            ax.clabel(contours, inline=True, fontsize=8, fmt='%d dB')
    
    if gnb_positions is not None:
        for i, (gx, gy) in enumerate(gnb_positions):
            ax.plot(gx, gy, 'k^', markersize=12, markeredgewidth=2,
                   markerfacecolor=GNB_COLORS[i % len(GNB_COLORS)])
    
    cbar = plt.colorbar(im, ax=ax, label='SINR (dB)', shrink=0.8)
    
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_best_server_map(df, title, output_path, gnb_positions=None):
    """Plot best server (cell) map showing dominant gNB at each point."""
    fig, ax = plt.subplots(figsize=(8, 7))
    
    grid, x_vals, y_vals = create_heatmap_grid(df, 'best_gnb_id')
    
    # Get unique valid gNB IDs (exclude -1 which means no coverage)
    unique_gnbs = sorted([x for x in df['best_gnb_id'].unique() if x >= 0])
    
    # Handle edge cases
    if len(unique_gnbs) == 0:
        print(f"  WARNING: No valid gNB IDs in data for {title}")
        ax.text(0.5, 0.5, 'No Coverage Data', ha='center', va='center', 
                transform=ax.transAxes, fontsize=14)
        plt.savefig(output_path)
        plt.close()
        return
    
    if len(unique_gnbs) == 1:
        # Single gNB case - use simple coloring
        print(f"  Note: Single gNB ({unique_gnbs[0]}) - using uniform coloring")
        colors = [GNB_COLORS[unique_gnbs[0] % len(GNB_COLORS)]]
        cmap = mcolors.ListedColormap(colors)
        
        # Replace invalid values with the single gNB ID for visualization
        grid_plot = np.where(np.isnan(grid), -1, grid)
        grid_plot = np.where(grid_plot < 0, np.nan, grid_plot)
        
        extent = [x_vals.min(), x_vals.max(), y_vals.min(), y_vals.max()]
        im = ax.imshow(grid_plot, extent=extent, origin='lower',
                       cmap=cmap, vmin=unique_gnbs[0]-0.5, vmax=unique_gnbs[0]+0.5, 
                       aspect='equal')
        
        # Legend for single gNB
        legend_elements = [Patch(facecolor=colors[0], edgecolor='black',
                                label=f'gNB {unique_gnbs[0]}')]
        ax.legend(handles=legend_elements, loc='upper right')
        
    else:
        # Multiple gNBs - use discrete colormap with boundaries
        num_gnbs = max(unique_gnbs) + 1
        colors = GNB_COLORS[:num_gnbs]
        cmap = mcolors.ListedColormap(colors)
        
        # Create bounds that span all possible gNB IDs
        bounds = np.arange(-0.5, num_gnbs + 0.5, 1)
        norm = mcolors.BoundaryNorm(bounds, cmap.N)
        
        extent = [x_vals.min(), x_vals.max(), y_vals.min(), y_vals.max()]
        
        # Replace NaN with -1 for plotting (will be outside colormap range)
        grid_plot = np.nan_to_num(grid, nan=-1)
        
        im = ax.imshow(grid_plot, extent=extent, origin='lower',
                       cmap=cmap, norm=norm, aspect='equal')
        
        # Add cell boundaries
        X, Y = np.meshgrid(x_vals, y_vals)
        boundary_levels = [i + 0.5 for i in range(num_gnbs - 1)]
        if boundary_levels:
            ax.contour(X, Y, grid_plot, levels=boundary_levels,
                       colors='black', linewidths=1.5)
        
        # Legend
        legend_elements = [Patch(facecolor=colors[i], edgecolor='black',
                                label=f'gNB {i}') for i in unique_gnbs]
        ax.legend(handles=legend_elements, loc='upper right')
    
    if gnb_positions is not None:
        for i, (gx, gy) in enumerate(gnb_positions):
            ax.plot(gx, gy, 'k^', markersize=14, markeredgewidth=2,
                   markerfacecolor='white')
            ax.annotate(f'{i}', (gx, gy), ha='center', va='center',
                       fontsize=FONT_SIZES['annotation'], fontweight='bold')
    
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_coverage_comparison(data_dict, output_path):
    """Compare coverage statistics across deployments."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    
    deployments = []
    coverage_data = {'excellent': [], 'good': [], 'fair': [], 'poor': [], 'none': []}
    sinr_means = []
    signal_means = []
    
    for name, df in sorted(data_dict.items()):
        if 'best_server' in name and 'stats' not in name:
            # Extract deployment name more robustly
            parts = name.split('_')
            dep_name = parts[1] if len(parts) > 1 else name
            deployments.append(dep_name)
            
            total = len(df)
            if total == 0:
                continue
                
            coverage_data['excellent'].append(len(df[df['best_signal_dbm'] >= -70]) / total * 100)
            coverage_data['good'].append(len(df[(df['best_signal_dbm'] >= -85) & (df['best_signal_dbm'] < -70)]) / total * 100)
            coverage_data['fair'].append(len(df[(df['best_signal_dbm'] >= -100) & (df['best_signal_dbm'] < -85)]) / total * 100)
            coverage_data['poor'].append(len(df[(df['best_signal_dbm'] >= -115) & (df['best_signal_dbm'] < -100)]) / total * 100)
            coverage_data['none'].append(len(df[df['best_signal_dbm'] < -115]) / total * 100)
            
            signal_means.append(df['best_signal_dbm'].mean())
            if 'sinr_db' in df.columns:
                sinr_means.append(df['sinr_db'].mean())
            else:
                sinr_means.append(0)
    
    if not deployments:
        print("  No deployment data found for coverage comparison")
        plt.close()
        return
    
    # Plot 1: Stacked bar chart of coverage
    ax1 = axes[0]
    x = np.arange(len(deployments))
    width = 0.6
    
    bottom = np.zeros(len(deployments))
    colors_coverage = ['#2ecc71', '#27ae60', '#f39c12', '#e74c3c', '#95a5a6']
    labels_coverage = ['Excellent (≥-70)', 'Good (≥-85)', 'Fair (≥-100)', 'Poor (≥-115)', 'No Coverage']
    
    for i, (cat, color) in enumerate(zip(['excellent', 'good', 'fair', 'poor', 'none'], colors_coverage)):
        ax1.bar(x, coverage_data[cat], width, bottom=bottom, label=labels_coverage[i], color=color)
        bottom += np.array(coverage_data[cat])
    
    ax1.set_ylabel('Coverage Distribution (%)')
    ax1.set_xlabel('Deployment Configuration')
    ax1.set_xticks(x)
    ax1.set_xticklabels(deployments, rotation=45, ha='right')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.set_title('Coverage Quality Distribution')
    
    # Plot 2: Mean signal strength
    ax2 = axes[1]
    bars = ax2.bar(x, signal_means, width, color='steelblue', edgecolor='black')
    ax2.axhline(y=-85, color='green', linestyle='--', label='Good threshold')
    ax2.axhline(y=-100, color='orange', linestyle='--', label='Fair threshold')
    ax2.set_ylabel('Mean Signal Strength (dBm)')
    ax2.set_xlabel('Deployment Configuration')
    ax2.set_xticks(x)
    ax2.set_xticklabels(deployments, rotation=45, ha='right')
    ax2.legend(loc='lower right', fontsize=8)
    ax2.set_title('Mean Signal Strength by Deployment')
    
    # Plot 3: Mean SINR
    ax3 = axes[2]
    bars = ax3.bar(x, sinr_means, width, color='coral', edgecolor='black')
    ax3.axhline(y=10, color='green', linestyle='--', label='Good SINR')
    ax3.axhline(y=0, color='red', linestyle='--', label='Interference zone')
    ax3.set_ylabel('Mean SINR (dB)')
    ax3.set_xlabel('Deployment Configuration')
    ax3.set_xticks(x)
    ax3.set_xticklabels(deployments, rotation=45, ha='right')
    ax3.legend(loc='lower right', fontsize=8)
    ax3.set_title('Mean SINR by Deployment')
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_interference_analysis(df, output_path):
    """Plot interference analysis: SINR vs gNB separation."""
    if df is None or df.empty:
        print("  No interference data available")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Plot 1: SINR vs Separation
    ax1 = axes[0]
    ax1.plot(df['separation_m'], df['mean_sinr_db'], 'o-', color='steelblue',
            markersize=8, linewidth=2, label='Mean SINR')
    ax1.plot(df['separation_m'], df['min_sinr_db'], 's--', color='coral',
            markersize=6, linewidth=1.5, label='Minimum SINR')
    ax1.axhline(y=0, color='red', linestyle=':', alpha=0.7, label='Interference threshold')
    ax1.axhline(y=10, color='green', linestyle=':', alpha=0.7, label='Good SINR')
    
    ax1.set_xlabel('gNB Separation Distance (m)')
    ax1.set_ylabel('SINR (dB)')
    ax1.set_title('SINR vs gNB Separation')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Interference zone percentage
    ax2 = axes[1]
    ax2.bar(df['separation_m'], df['interference_zone_pct'], width=3,
           color='indianred', edgecolor='black', alpha=0.8)
    ax2.set_xlabel('gNB Separation Distance (m)')
    ax2.set_ylabel('Interference Zone (%)')
    ax2.set_title('Percentage of Area with SINR < 5 dB')
    ax2.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_coverage_complementarity(df, output_path):
    """Plot coverage complementarity analysis."""
    if df is None or df.empty:
        print("  No complementarity data available")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Plot 1: Coverage vs number of gNBs
    ax1 = axes[0]
    x = df['num_gnbs']
    ax1.plot(x, df['coverage_total_pct'], 'o-', color='steelblue',
            markersize=10, linewidth=2, label='Total Coverage')
    ax1.bar(x, df['coverage_gain_pct'], width=0.3, color='lightgreen',
           edgecolor='black', alpha=0.7, label='Incremental Gain')
    
    ax1.set_xlabel('Number of gNBs')
    ax1.set_ylabel('Coverage (%)')
    ax1.set_title('Coverage vs Number of gNBs')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xticks(x)
    
    # Plot 2: Overlap percentage
    ax2 = axes[1]
    ax2.bar(x, df['overlap_pct'], width=0.5, color='coral', edgecolor='black')
    ax2.set_xlabel('Number of gNBs')
    ax2.set_ylabel('Overlap Area (%)')
    ax2.set_title('Multi-gNB Coverage Overlap')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_xticks(x)
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def plot_cell_boundary_analysis(df, output_path):
    """Plot cell boundary / handover region analysis."""
    if df is None or df.empty or len(df) == 0:
        print("  No cell boundary data available")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Plot 1: Scatter plot of boundary points
    ax1 = axes[0]
    scatter = ax1.scatter(df['x'], df['y'], c=df['margin_db'], cmap='RdYlGn',
                          s=20, alpha=0.7, vmin=0, vmax=6)
    plt.colorbar(scatter, ax=ax1, label='Handover Margin (dB)')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Cell Boundary Regions (Handover Margin < 6 dB)')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Histogram of handover margins
    ax2 = axes[1]
    ax2.hist(df['margin_db'], bins=20, color='steelblue', edgecolor='black', alpha=0.7)
    ax2.axvline(x=3, color='red', linestyle='--', label='Critical margin (3 dB)')
    ax2.set_xlabel('Handover Margin (dB)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Distribution of Handover Margins')
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"  Saved: {output_path}")


def generate_summary_statistics(data_dict, output_path):
    """Generate summary statistics CSV for all deployments."""
    summary_data = []
    
    for name, df in data_dict.items():
        if 'stats' not in name and len(df) > 0:
            stats = {
                'deployment': name,
                'num_points': len(df),
                'mean_signal_dbm': df['best_signal_dbm'].mean() if 'best_signal_dbm' in df.columns else np.nan,
                'std_signal_dbm': df['best_signal_dbm'].std() if 'best_signal_dbm' in df.columns else np.nan,
                'min_signal_dbm': df['best_signal_dbm'].min() if 'best_signal_dbm' in df.columns else np.nan,
                'max_signal_dbm': df['best_signal_dbm'].max() if 'best_signal_dbm' in df.columns else np.nan,
            }
            
            if 'sinr_db' in df.columns:
                stats['mean_sinr_db'] = df['sinr_db'].mean()
                stats['std_sinr_db'] = df['sinr_db'].std()
                stats['min_sinr_db'] = df['sinr_db'].min()
            
            if 'best_signal_dbm' in df.columns:
                total = len(df)
                stats['coverage_excellent_pct'] = len(df[df['best_signal_dbm'] >= -70]) / total * 100
                stats['coverage_good_pct'] = len(df[(df['best_signal_dbm'] >= -85) & (df['best_signal_dbm'] < -70)]) / total * 100
                stats['coverage_fair_pct'] = len(df[(df['best_signal_dbm'] >= -100) & (df['best_signal_dbm'] < -85)]) / total * 100
                stats['coverage_total_pct'] = len(df[df['best_signal_dbm'] >= -100]) / total * 100
            
            summary_data.append(stats)
    
    if summary_data:
        summary_df = pd.DataFrame(summary_data)
        summary_df.to_csv(output_path, index=False)
        print(f"  Saved: {output_path}")
    else:
        print("  No data to summarize")


def diagnose_data(data_dict):
    """Diagnose potential data issues."""
    print("\n" + "="*60)
    print("  DATA DIAGNOSTICS")
    print("="*60 + "\n")
    
    for name, df in data_dict.items():
        print(f"File: {name}")
        print(f"  Shape: {df.shape}")
        print(f"  Columns: {list(df.columns)}")
        
        if 'best_signal_dbm' in df.columns:
            sig = df['best_signal_dbm']
            print(f"  Signal: min={sig.min():.1f}, max={sig.max():.1f}, mean={sig.mean():.1f}")
            print(f"  Signal -150 count: {len(df[sig <= -149])} / {len(df)}")
        
        if 'best_gnb_id' in df.columns:
            print(f"  gNB IDs: {sorted(df['best_gnb_id'].unique())}")
        
        if 'sinr_db' in df.columns:
            sinr = df['sinr_db']
            print(f"  SINR: min={sinr.min():.1f}, max={sinr.max():.1f}, mean={sinr.mean():.1f}")
        
        print()


def main(results_dir):
    """Main analysis function."""
    print(f"\n{'='*60}")
    print("  Multi-gNB Deployment Analysis")
    print(f"{'='*60}\n")
    print(f"Results directory: {results_dir}\n")
    
    # Setup output directories
    figures_dir = os.path.join(results_dir, 'figures')
    os.makedirs(figures_dir, exist_ok=True)
    
    # Load all data
    print("Loading data...")
    data = load_all_results(results_dir)
    print(f"  Found {len(data)} data files\n")
    
    if not data:
        print("ERROR: No data files found!")
        return
    
    # Run diagnostics
    diagnose_data(data)
    
    # Generate heatmaps for each deployment
    print("Generating heatmaps...")
    for name, df in data.items():
        if len(df) == 0:
            print(f"  Skipping {name} - empty dataframe")
            continue
            
        if 'best_signal_dbm' in df.columns:
            plot_signal_heatmap(df, f'Signal Strength: {name}',
                              os.path.join(figures_dir, f'{name}_signal_heatmap.pdf'))
        
        if 'sinr_db' in df.columns and df['sinr_db'].notna().any():
            plot_sinr_heatmap(df, f'SINR: {name}',
                            os.path.join(figures_dir, f'{name}_sinr_heatmap.pdf'))
        
        if 'best_gnb_id' in df.columns:
            plot_best_server_map(df, f'Best Server: {name}',
                               os.path.join(figures_dir, f'{name}_best_server.pdf'))
    
    # Coverage comparison
    print("\nGenerating comparison plots...")
    plot_coverage_comparison(data, os.path.join(figures_dir, 'coverage_comparison.pdf'))
    
    # Load and plot interference analysis
    interf_file = os.path.join(results_dir, 'raw_data', 'interference_vs_separation.csv')
    if os.path.exists(interf_file):
        interf_df = pd.read_csv(interf_file)
        plot_interference_analysis(interf_df, os.path.join(figures_dir, 'interference_analysis.pdf'))
    
    # Load and plot complementarity analysis
    comp_file = os.path.join(results_dir, 'raw_data', 'coverage_complementarity.csv')
    if os.path.exists(comp_file):
        comp_df = pd.read_csv(comp_file)
        plot_coverage_complementarity(comp_df, os.path.join(figures_dir, 'coverage_complementarity.pdf'))
    
    # Generate summary statistics
    print("\nGenerating summary statistics...")
    generate_summary_statistics(data, os.path.join(results_dir, 'analysis', 'summary_statistics.csv'))
    
    print(f"\n{'='*60}")
    print("  Analysis Complete!")
    print(f"{'='*60}")
    print(f"\nFigures saved to: {figures_dir}")
    print(f"Analysis saved to: {os.path.join(results_dir, 'analysis')}\n")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python analyze_multi_gnb.py <results_directory>")
        print("Example: python analyze_multi_gnb.py multi_gnb_results/20241215_120000")
        sys.exit(1)
    
    results_dir = sys.argv[1]
    
    if not os.path.isdir(results_dir):
        print(f"Error: Directory not found: {results_dir}")
        sys.exit(1)
    
    main(results_dir)
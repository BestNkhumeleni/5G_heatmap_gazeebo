#!/usr/bin/env python3
"""
RF Propagation Model Comparison Analysis
Generates publication-quality figures and statistical analysis for conference paper.
"""

import os
import sys
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

# Configure matplotlib for publication quality - matched to heatmap sizing
plt.rcParams.update({
    'font.size': 20,
    'font.family': 'serif',
    'font.weight': 'bold',
    'axes.labelsize': 24,
    'axes.labelweight': 'bold',
    'axes.titlesize': 32,
    'axes.titleweight': 'bold',
    'legend.fontsize': 22,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'figure.figsize': (16, 10),
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.linewidth': 2,
    'xtick.major.width': 2,
    'ytick.major.width': 2,
    'xtick.major.size': 6,
    'ytick.major.size': 6,
    'lines.linewidth': 2.5,
    'lines.markersize': 10,
})

# Model display names and colors
MODEL_CONFIG = {
    'free_space': {'name': 'FSPL', 'color': '#2ecc71', 'marker': 'o'},
    '3gpp_umi': {'name': '3GPP UMi', 'color': '#3498db', 'marker': 's'},
    '3gpp_uma': {'name': '3GPP UMa', 'color': '#9b59b6', 'marker': '^'},
    'ray_tracing': {'name': 'Ray Tracing', 'color': '#e74c3c', 'marker': 'D'},
    'hybrid': {'name': 'Hybrid', 'color': '#f39c12', 'marker': 'v'},
}

SCENARIO_CONFIG = {
    'scenario_open_field': {'name': 'Open Field', 'short': 'OF'},
    'scenario_dense_urban': {'name': 'Dense Urban', 'short': 'DU'},
    'scenario_street_canyon': {'name': 'Street Canyon', 'short': 'SC'},
    'scenario_elevated_macro': {'name': 'Elevated Macro', 'short': 'EM'},
    'scenario_indoor_outdoor': {'name': 'Indoor-Outdoor', 'short': 'IO'},
}

# Signal quality thresholds
THRESHOLDS = {
    'excellent': -70,
    'good': -85,
    'fair': -100,
    'poor': -115,
}


def load_raw_data(results_dir):
    """Load all raw CSV data files."""
    data_frames = []
    raw_dir = Path(results_dir) / 'raw_data'
    
    for csv_file in raw_dir.glob('scenario_*.csv'):
        if '_stats' not in csv_file.name:
            scenario = csv_file.stem
            df = pd.read_csv(csv_file)
            df['scenario'] = scenario
            data_frames.append(df)
    
    if not data_frames:
        print(f"No data files found in {raw_dir}")
        return None
    
    return pd.concat(data_frames, ignore_index=True)


def load_summary_data(results_dir):
    """Load the summary CSV file."""
    summary_file = Path(results_dir) / 'summary_all_scenarios.csv'
    if summary_file.exists():
        return pd.read_csv(summary_file)
    return None


def plot_signal_vs_distance(df, output_dir):
    """Plot signal strength vs distance for each scenario."""
    fig, axes = plt.subplots(2, 3, figsize=(20, 14))
    axes = axes.flatten()
    
    scenarios = df['scenario'].unique()
    
    for idx, scenario in enumerate(scenarios):
        if idx >= len(axes):
            break
            
        ax = axes[idx]
        scenario_data = df[df['scenario'] == scenario]
        
        for model_key, config in MODEL_CONFIG.items():
            model_data = scenario_data[scenario_data['model'] == model_key]
            if not model_data.empty:
                ax.scatter(model_data['distance_m'], model_data['signal_dbm'],
                          label=config['name'], color=config['color'],
                          marker=config['marker'], alpha=0.8, s=120, edgecolors='black', linewidths=0.8)
        
        scenario_name = SCENARIO_CONFIG.get(scenario, {}).get('name', scenario)
        ax.set_title(scenario_name, fontweight='bold', fontsize=24)
        ax.set_xlabel('Distance (m)', fontweight='bold', fontsize=20)
        ax.set_ylabel('Signal Strength (dBm)', fontweight='bold', fontsize=20)
        ax.tick_params(axis='both', labelsize=16)
        ax.grid(True, alpha=0.3, linewidth=1.2)
        ax.set_ylim(-130, -30)
        
        # Add threshold lines
        for thresh_name, thresh_val in THRESHOLDS.items():
            ax.axhline(y=thresh_val, color='gray', linestyle='--', alpha=0.3)
    
    # Hide unused subplots
    for idx in range(len(scenarios), len(axes)):
        axes[idx].set_visible(False)
    
    # Add legend
    handles = [mpatches.Patch(color=c['color'], label=c['name']) 
               for c in MODEL_CONFIG.values()]
    fig.legend(handles=handles, loc='center right', bbox_to_anchor=(1.0, 0.5),
               fontsize=18, frameon=True, fancybox=True)
    
    plt.tight_layout(rect=[0, 0, 0.87, 1])
    plt.savefig(output_dir / 'signal_vs_distance.pdf')
    plt.savefig(output_dir / 'signal_vs_distance.png')
    plt.close()
    print(f"Saved: signal_vs_distance.pdf/png")


def plot_model_comparison_heatmap(summary_df, output_dir):
    """Create a heatmap comparing models across scenarios."""
    if summary_df is None:
        return
    
    # Pivot for mean signal strength
    pivot = summary_df.pivot(index='scenario', columns='model', values='mean_dbm')
    
    # Reorder columns and rows
    model_order = list(MODEL_CONFIG.keys())
    scenario_order = list(SCENARIO_CONFIG.keys())
    
    pivot = pivot.reindex(columns=[m for m in model_order if m in pivot.columns])
    pivot = pivot.reindex([s for s in scenario_order if s in pivot.index])
    
    # Rename for display
    pivot.columns = [MODEL_CONFIG[m]['name'] for m in pivot.columns]
    pivot.index = [SCENARIO_CONFIG[s]['name'] for s in pivot.index]
    
    fig, ax = plt.subplots(figsize=(16, 10))
    
    im = ax.imshow(pivot.values, cmap='RdYlGn', aspect='auto', vmin=-120, vmax=-50)
    
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_yticks(range(len(pivot.index)))
    ax.set_xticklabels(pivot.columns, rotation=45, ha='right', fontweight='bold', fontsize=24)
    ax.set_yticklabels(pivot.index, fontweight='bold', fontsize=24)
    
    # Add values
    for i in range(len(pivot.index)):
        for j in range(len(pivot.columns)):
            val = pivot.values[i, j]
            color = 'white' if val < -90 else 'black'
            ax.text(j, i, f'{val:.1f}', ha='center', va='center', color=color, 
                   fontsize=22, fontweight='bold')
    
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Mean Signal Strength (dBm)', fontweight='bold', fontsize=24)
    cbar.ax.tick_params(labelsize=20)
    ax.set_title('Propagation Model Performance Comparison', fontweight='bold', fontsize=32)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'model_comparison_heatmap.pdf')
    plt.savefig(output_dir / 'model_comparison_heatmap.png')
    plt.close()
    print(f"Saved: model_comparison_heatmap.pdf/png")


def plot_coverage_distribution(summary_df, output_dir):
    """Create stacked bar chart of coverage quality distribution."""
    if summary_df is None:
        return
    
    coverage_cols = ['excellent_pct', 'good_pct', 'fair_pct', 'poor_pct', 'no_service_pct']
    colors = ['#27ae60', '#3498db', '#f1c40f', '#e67e22', '#e74c3c']
    labels = ['Excellent', 'Good', 'Fair', 'Poor', 'No Service']
    
    scenarios = list(SCENARIO_CONFIG.keys())
    models = list(MODEL_CONFIG.keys())
    
    fig, axes = plt.subplots(1, len(scenarios), figsize=(22, 8), sharey=True)
    
    for s_idx, scenario in enumerate(scenarios):
        ax = axes[s_idx]
        scenario_data = summary_df[summary_df['scenario'] == scenario]
        
        x = range(len(models))
        bottoms = np.zeros(len(models))
        
        for c_idx, col in enumerate(coverage_cols):
            values = []
            for model in models:
                model_row = scenario_data[scenario_data['model'] == model]
                if not model_row.empty:
                    values.append(model_row[col].values[0])
                else:
                    values.append(0)
            
            ax.bar(x, values, bottom=bottoms, color=colors[c_idx], 
                   label=labels[c_idx] if s_idx == 0 else None, width=0.7, edgecolor='black', linewidth=0.8)
            bottoms += values
        
        ax.set_xticks(x)
        ax.set_xticklabels([MODEL_CONFIG[m]['name'] for m in models], 
                          rotation=45, ha='right', fontsize=16, fontweight='bold')
        ax.set_title(SCENARIO_CONFIG[scenario]['name'], fontsize=24, fontweight='bold')
        ax.set_ylim(0, 100)
        ax.tick_params(axis='y', labelsize=16)
        
        if s_idx == 0:
            ax.set_ylabel('Coverage (%)', fontweight='bold', fontsize=20)
    
    fig.legend(labels, loc='center right', bbox_to_anchor=(1.08, 0.5), 
               fontsize=18, frameon=True, fancybox=True)
    plt.suptitle('Coverage Quality Distribution by Model and Scenario', fontsize=28, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 0.91, 0.94])
    plt.savefig(output_dir / 'coverage_distribution.pdf')
    plt.savefig(output_dir / 'coverage_distribution.png')
    plt.close()
    print(f"Saved: coverage_distribution.pdf/png")


def plot_model_ranking(summary_df, output_dir):
    """Create ranking plot showing best model for each scenario."""
    if summary_df is None:
        return
    
    fig, ax = plt.subplots(figsize=(12, 7))
    
    scenarios = list(SCENARIO_CONFIG.keys())
    scenario_names = [SCENARIO_CONFIG[s]['name'] for s in scenarios]
    
    x = np.arange(len(scenarios))
    width = 0.15
    
    for m_idx, (model_key, config) in enumerate(MODEL_CONFIG.items()):
        means = []
        stds = []
        for scenario in scenarios:
            row = summary_df[(summary_df['scenario'] == scenario) & 
                            (summary_df['model'] == model_key)]
            if not row.empty:
                means.append(row['mean_dbm'].values[0])
                stds.append(row['std_dbm'].values[0])
            else:
                means.append(np.nan)
                stds.append(0)
        
        offset = (m_idx - len(MODEL_CONFIG)/2 + 0.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds, capsize=3,
                     label=config['name'], color=config['color'], alpha=0.85,
                     edgecolor='black', linewidth=0.5, error_kw={'linewidth': 1.5})
    
    ax.set_xticks(x)
    ax.set_xticklabels(scenario_names, rotation=30, ha='right', fontweight='bold', fontsize=12)
    ax.set_ylabel('Mean Signal Strength (dBm)', fontweight='bold', fontsize=14)
    ax.set_title('Model Performance Comparison Across Scenarios', fontweight='bold', fontsize=16)
    ax.legend(loc='upper right', fontsize=11, frameon=True, fancybox=True)
    ax.grid(True, axis='y', alpha=0.3, linewidth=1)
    ax.tick_params(axis='y', labelsize=11)
    
    # Add threshold reference
    ax.axhline(y=-85, color='green', linestyle='--', alpha=0.6, linewidth=2, label='Good threshold')
    ax.axhline(y=-100, color='orange', linestyle='--', alpha=0.6, linewidth=2, label='Fair threshold')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'model_ranking.pdf')
    plt.savefig(output_dir / 'model_ranking.png')
    plt.close()
    print(f"Saved: model_ranking.pdf/png")


def generate_statistics_table(summary_df, output_dir):
    """Generate detailed statistics table for paper."""
    if summary_df is None:
        return
    
    # Create formatted table
    table_data = []
    
    for scenario in SCENARIO_CONFIG.keys():
        scenario_data = summary_df[summary_df['scenario'] == scenario]
        for model in MODEL_CONFIG.keys():
            row = scenario_data[scenario_data['model'] == model]
            if not row.empty:
                table_data.append({
                    'Scenario': SCENARIO_CONFIG[scenario]['name'],
                    'Model': MODEL_CONFIG[model]['name'],
                    'Mean (dBm)': f"{row['mean_dbm'].values[0]:.1f}",
                    'Std (dB)': f"{row['std_dbm'].values[0]:.1f}",
                    'Range (dBm)': f"[{row['min_dbm'].values[0]:.0f}, {row['max_dbm'].values[0]:.0f}]",
                    'Good+ (%)': f"{row['excellent_pct'].values[0] + row['good_pct'].values[0]:.0f}",
                })
    
    df_table = pd.DataFrame(table_data)
    
    # Save as CSV
    df_table.to_csv(output_dir / 'statistics_table.csv', index=False)
    
    # Save as LaTeX
    latex_table = df_table.to_latex(index=False, escape=False)
    with open(output_dir / 'statistics_table.tex', 'w') as f:
        f.write(latex_table)
    
    print(f"Saved: statistics_table.csv/tex")


def identify_best_models(summary_df, output_dir):
    """Identify and report the best model for each scenario."""
    if summary_df is None:
        return
    
    report_lines = ["Best Model Recommendations by Scenario", "=" * 50, ""]
    
    for scenario in SCENARIO_CONFIG.keys():
        scenario_data = summary_df[summary_df['scenario'] == scenario]
        if scenario_data.empty:
            continue
        
        # Best by mean signal
        best_mean_idx = scenario_data['mean_dbm'].idxmax()
        best_mean = scenario_data.loc[best_mean_idx]
        
        # Best by coverage (excellent + good)
        scenario_data = scenario_data.copy()
        scenario_data['good_coverage'] = scenario_data['excellent_pct'] + scenario_data['good_pct']
        best_cov_idx = scenario_data['good_coverage'].idxmax()
        best_cov = scenario_data.loc[best_cov_idx]
        
        scenario_name = SCENARIO_CONFIG[scenario]['name']
        report_lines.append(f"\n{scenario_name}:")
        report_lines.append(f"  Best mean signal: {MODEL_CONFIG[best_mean['model']]['name']} "
                          f"({best_mean['mean_dbm']:.1f} dBm)")
        report_lines.append(f"  Best coverage:    {MODEL_CONFIG[best_cov['model']]['name']} "
                          f"({best_cov['good_coverage']:.0f}% good+)")
    
    report = '\n'.join(report_lines)
    print(report)
    
    with open(output_dir / 'best_models_report.txt', 'w') as f:
        f.write(report)
    
    print(f"\nSaved: best_models_report.txt")


def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_results.py <results_directory>")
        print("Example: python analyze_results.py comparison_results/20241215_143022")
        sys.exit(1)
    
    results_dir = Path(sys.argv[1])
    
    if not results_dir.exists():
        print(f"Directory not found: {results_dir}")
        sys.exit(1)
    
    # Create figures directory
    figures_dir = results_dir / 'figures'
    figures_dir.mkdir(exist_ok=True)
    
    print(f"Analyzing results in: {results_dir}")
    print(f"Saving figures to: {figures_dir}")
    print("=" * 50)
    
    # Load data
    raw_df = load_raw_data(results_dir)
    summary_df = load_summary_data(results_dir)
    
    if raw_df is not None:
        print(f"Loaded {len(raw_df)} raw data points")
        plot_signal_vs_distance(raw_df, figures_dir)
    
    if summary_df is not None:
        print(f"Loaded summary data with {len(summary_df)} rows")
        plot_model_comparison_heatmap(summary_df, figures_dir)
        plot_coverage_distribution(summary_df, figures_dir)
        plot_model_ranking(summary_df, figures_dir)
        generate_statistics_table(summary_df, figures_dir)
        identify_best_models(summary_df, figures_dir)
    
    print("\n" + "=" * 50)
    print("Analysis complete!")


if __name__ == '__main__':
    main()
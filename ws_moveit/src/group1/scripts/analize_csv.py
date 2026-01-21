#!/usr/bin/env python3
"""
Benchmark Results Analyzer

Analyzes planner benchmark CSV results and generates:
- Success rate statistics
- Planning time comparisons
- Path length analysis
- Shortcut effectiveness
- Statistical summaries by environment

Usage:
  python3 analyze_benchmark.py <csv_file>
  python3 analyze_benchmark.py ~/benchmarks/benchmark_results_20260121_180831.csv
"""

import sys
import pandas as pd
import numpy as np
from pathlib import Path


def load_data(csv_path):
    """Load and validate benchmark CSV data."""
    try:
        df = pd.read_csv(csv_path)
        print(f"✅ Loaded {len(df)} rows from {csv_path}\n")
        return df
    except FileNotFoundError:
        print(f"❌ Error: File not found: {csv_path}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error loading CSV: {e}")
        sys.exit(1)


def print_section(title):
    """Print formatted section header."""
    print(f"\n{'='*70}")
    print(f"  {title}")
    print(f"{'='*70}\n")


def basic_statistics(df):
    """Print basic dataset statistics."""
    print_section("DATASET OVERVIEW")
    
    print(f"Total planning attempts: {len(df)}")
    print(f"Total runs per environment: {df['run'].max()}")
    print(f"Environments: {', '.join(df['environment'].unique())}")
    print(f"Planners: {', '.join(df['planner'].unique())}")
    print(f"Date range: {df['timestamp'].min()} to {df['timestamp'].max()}")


def success_rate_analysis(df):
    """Analyze success rates by planner and environment."""
    print_section("SUCCESS RATE ANALYSIS")
    
    # Overall success rates by planner
    success_by_planner = df.groupby('planner').agg({
        'success': ['sum', 'count', 'mean']
    }).round(4)
    success_by_planner.columns = ['Successes', 'Total', 'Success Rate']
    success_by_planner['Success Rate %'] = (success_by_planner['Success Rate'] * 100).round(2)
    
    print("Overall Success Rates by Planner:")
    print(success_by_planner[['Successes', 'Total', 'Success Rate %']])
    
    # Success rates by environment and planner
    print("\n\nSuccess Rates by Environment:")
    pivot = df.pivot_table(
        values='success',
        index='planner',
        columns='environment',
        aggfunc='mean'
    ) * 100
    print(pivot.round(2))
    
    # Find best performer per environment
    print("\n\nBest Planner per Environment (by success rate):")
    for env in df['environment'].unique():
        env_data = df[df['environment'] == env]
        best = env_data.groupby('planner')['success'].mean().idxmax()
        rate = env_data.groupby('planner')['success'].mean().max() * 100
        print(f"  {env:20s}: {best:15s} ({rate:.2f}%)")


def planning_time_analysis(df):
    """Analyze planning times."""
    print_section("PLANNING TIME ANALYSIS")
    
    # Filter only successful plans
    successful = df[df['success'] == True]
    
    if len(successful) == 0:
        print("❌ No successful plans to analyze")
        return
    
    # Statistics by planner
    time_stats = successful.groupby('planner')['planning_time_sec'].agg([
        'count', 'mean', 'std', 'min', 'median', 'max'
    ]).round(4)
    
    print("Planning Time Statistics (successful plans only):")
    print(time_stats)
    
    # By environment
    print("\n\nAverage Planning Time by Environment:")
    env_time = successful.pivot_table(
        values='planning_time_sec',
        index='planner',
        columns='environment',
        aggfunc='mean'
    ).round(4)
    print(env_time)
    
    # Find fastest planner per environment
    print("\n\nFastest Planner per Environment:")
    for env in successful['environment'].unique():
        env_data = successful[successful['environment'] == env]
        fastest = env_data.groupby('planner')['planning_time_sec'].mean().idxmin()
        time = env_data.groupby('planner')['planning_time_sec'].mean().min()
        print(f"  {env:20s}: {fastest:15s} ({time:.4f}s)")


def path_length_analysis(df):
    """Analyze path lengths."""
    print_section("PATH LENGTH ANALYSIS")
    
    successful = df[(df['success'] == True) & (df['path_length'] != float('inf'))]
    
    if len(successful) == 0:
        print("No successful plans with valid path lengths")
        return
    
    length_stats = successful.groupby('planner')['path_length'].agg([
        'count', 'mean', 'std', 'min', 'median', 'max'
    ]).round(6)
    
    print("Path Length Statistics (successful plans only):")
    print(length_stats)
    
    print("\n\nAverage Path Length by Environment:")
    env_length = successful.pivot_table(
        values='path_length',
        index='planner',
        columns='environment',
        aggfunc='mean'
    ).round(6)
    print(env_length)
    
    print("\n\nShortest Path Planner per Environment:")
    for env in successful['environment'].unique():
        env_data = successful[successful['environment'] == env]
        shortest = env_data.groupby('planner')['path_length'].mean().idxmin()
        length = env_data.groupby('planner')['path_length'].mean().min()
        print(f"  {env:20s}: {shortest:15s} ({length:.6f})")


def waypoint_analysis(df):
    """Analyze waypoint counts."""
    print_section("WAYPOINT COUNT ANALYSIS")
    
    successful = df[(df['success'] == True) & (df['waypoint_count'] > 0)]
    
    if len(successful) == 0:
        print("No successful plans with waypoints")
        return
    
    waypoint_stats = successful.groupby('planner')['waypoint_count'].agg([
        'count', 'mean', 'std', 'min', 'median', 'max'
    ]).round(2)
    
    print("Waypoint Count Statistics:")
    print(waypoint_stats)
    
    print("\n\nAverage Waypoint Count by Environment:")
    env_waypoints = successful.pivot_table(
        values='waypoint_count',
        index='planner',
        columns='environment',
        aggfunc='mean'
    ).round(2)
    print(env_waypoints)


def shortcut_effectiveness(df):
    """Analyze shortcut effectiveness."""
    print_section("SHORTCUT EFFECTIVENESS ANALYSIS")
    
    # Filter planners with shortcuts
    shortcut_df = df[df['shortcut_applied'] == True]
    
    if len(shortcut_df) == 0:
        print("No shortcuts were applied")
        return
    
    print(f"Total shortcuts applied: {len(shortcut_df)}")
    print(f"Percentage of plans shortened: {len(shortcut_df)/len(df)*100:.2f}%\n")
    
    # Shortcut reduction
    reduction_stats = shortcut_df.groupby('planner')['shortcut_reduction'].agg([
        'count', 'mean', 'std', 'min', 'median', 'max'
    ]).round(6)
    
    print("Shortcut Reduction Statistics:")
    print(reduction_stats)
    
    print("\n\nShortcut Impact by Environment:")
    for env in shortcut_df['environment'].unique():
        env_data = shortcut_df[shortcut_df['environment'] == env]
        for planner in env_data['planner'].unique():
            planner_data = env_data[env_data['planner'] == planner]
            avg_reduction = planner_data['shortcut_reduction'].mean()
            count = len(planner_data)
            print(f"  {env:20s} | {planner:15s}: {avg_reduction:.6f} avg reduction ({count} times)")


def motion_comparison(df):
    """Compare performance across different motions."""
    print_section("MOTION-SPECIFIC ANALYSIS")
    
    print("Success Rate by Motion:")
    motion_success = df.pivot_table(
        values='success',
        index='planner',
        columns='motion',
        aggfunc='mean'
    ) * 100
    print(motion_success.round(2))
    
    print("\n\nAverage Planning Time by Motion (successful only):")
    successful = df[df['success'] == True]
    motion_time = successful.pivot_table(
        values='planning_time_sec',
        index='planner',
        columns='motion',
        aggfunc='mean'
    )
    print(motion_time.round(4))


def execution_failures(df):
    """Analyze execution failures."""
    print_section("EXECUTION ANALYSIS")
    
    total = len(df)
    skipped = df['execution_skipped'].sum()
    
    print(f"Total execution attempts: {total}")
    print(f"Executions skipped: {skipped} ({skipped/total*100:.2f}%)")
    
    if skipped > 0:
        print("\n\nExecution Skips by Environment:")
        skip_by_env = df.groupby('environment')['execution_skipped'].agg(['sum', 'count'])
        skip_by_env['percentage'] = (skip_by_env['sum'] / skip_by_env['count'] * 100).round(2)
        print(skip_by_env)


def generate_summary_table(df):
    """Generate overall comparison table."""
    print_section("OVERALL PLANNER COMPARISON")
    
    successful = df[df['success'] == True]
    
    summary = pd.DataFrame()
    
    for planner in df['planner'].unique():
        planner_data = df[df['planner'] == planner]
        planner_success = successful[successful['planner'] == planner]
        
        summary = pd.concat([summary, pd.DataFrame({
            'Planner': [planner],
            'Success Rate %': [planner_data['success'].mean() * 100],
            'Avg Time (s)': [planner_success['planning_time_sec'].mean() if len(planner_success) > 0 else np.nan],
            'Avg Path Length': [planner_success['path_length'].mean() if len(planner_success) > 0 else np.nan],
            'Avg Waypoints': [planner_success['waypoint_count'].mean() if len(planner_success) > 0 else np.nan],
        })], ignore_index=True)
    
    summary = summary.round(4)
    print(summary.to_string(index=False))


def export_summary_csv(df, output_path):
    """Export summary statistics to CSV."""
    successful = df[df['success'] == True]
    
    summary_rows = []
    
    for env in df['environment'].unique():
        for planner in df['planner'].unique():
            env_planner = df[(df['environment'] == env) & (df['planner'] == planner)]
            env_planner_success = successful[(successful['environment'] == env) & (successful['planner'] == planner)]
            
            summary_rows.append({
                'environment': env,
                'planner': planner,
                'total_attempts': len(env_planner),
                'successes': env_planner['success'].sum(),
                'success_rate': env_planner['success'].mean(),
                'avg_planning_time': env_planner_success['planning_time_sec'].mean() if len(env_planner_success) > 0 else np.nan,
                'avg_path_length': env_planner_success['path_length'].mean() if len(env_planner_success) > 0 else np.nan,
                'avg_waypoints': env_planner_success['waypoint_count'].mean() if len(env_planner_success) > 0 else np.nan,
            })
    
    summary_df = pd.DataFrame(summary_rows)
    summary_df.to_csv(output_path, index=False)
    print(f"\n Summary exported to: {output_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_benchmark.py <csv_file>")
        print("Example: python3 analyze_benchmark.py ~/benchmarks/benchmark_results_20260121_180831.csv")
        sys.exit(1)
    
    csv_path = sys.argv[1]
    df = load_data(csv_path)

    basic_statistics(df)
    success_rate_analysis(df)
    planning_time_analysis(df)
    path_length_analysis(df)
    waypoint_analysis(df)
    shortcut_effectiveness(df)
    motion_comparison(df)
    execution_failures(df)
    generate_summary_table(df)
    
    output_dir = Path(csv_path).parent
    summary_path = output_dir / f"summary_{Path(csv_path).stem}.csv"
    export_summary_csv(df, summary_path)
    
    print(f"\n{'='*70}")
    print("  ANALYSIS COMPLETE!")
    print(f"{'='*70}\n")


if __name__ == "__main__":
    main()

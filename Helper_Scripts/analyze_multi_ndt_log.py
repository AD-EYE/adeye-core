#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Iterate Through Multiple Folders to Parse NDT Log Files and Aggregate Metrics

Usage example:
    python analyze_multi_ndt_log.py /disk2/logs/ --keyword 25_13_20

This script:
  - Recursively searches for subdirectories in the specified root directory.
  - If --keyword is specified, only folders with the keyword in their name are processed.
  - In each folder, the log file "ndt_matching-13-stdout.log" is parsed.
  - Descriptive statistics (mean, std, min, max, count) for key metrics are printed.
  - The voxel size is extracted from the folder name (expected in the format "voxel_leaf_X").
  - An aggregated plot is generated with error bars:
       - X-axis: Voxel Size
       - Y-axis (subplot 1): Mean Fitness Score with error bars (std)
       - Y-axis (subplot 2): Mean Align Time with error bars (std)
"""

import os
import re
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def parse_log_file(file_path):
    """
    Parse the log file and extract key fields from each block.

    Returns:
        A pandas DataFrame with columns:
            Timestamp, FitnessScore, FilteredScanPoints, NDTConverged,
            NumIterations, NDTReliability, AlignTime, TransformationMatrix
    """
    try:
        import io
        with io.open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print("Error opening file {}: {}".format(file_path, e))
        return pd.DataFrame()

    # Split content into blocks using lines with at least 5 dashes as delimiters
    blocks = re.split(r'-{5,}', content)
    records = []
    for block in blocks:
        if 'Sequence:' not in block:
            continue
        record = {}
        # Timestamp
        m = re.search(r'Timestamp:\s*([\d\.]+)', block)
        record['Timestamp'] = float(m.group(1)) if m else np.nan
        # Fitness Score
        m = re.search(r'Fitness Score:\s*([\d\.]+)', block)
        record['FitnessScore'] = float(m.group(1)) if m else np.nan
        # Filtered Scan Points
        m = re.search(r'Number of Filtered Scan Points:\s*(\d+)', block)
        record['FilteredScanPoints'] = int(m.group(1)) if m else np.nan
        # NDT Convergence flag
        m = re.search(r'NDT has converged:\s*(\d+)', block)
        record['NDTConverged'] = int(m.group(1)) if m else np.nan
        # Number of Iterations
        m = re.search(r'Number of Iterations:\s*(\d+)', block)
        record['NumIterations'] = int(m.group(1)) if m else np.nan
        # NDT Reliability
        m = re.search(r'NDT Reliability:\s*([\d\.]+)', block)
        record['NDTReliability'] = float(m.group(1)) if m else np.nan
        # Align Time
        m = re.search(r'Align time:\s*([\d\.]+)', block)
        record['AlignTime'] = float(m.group(1)) if m else np.nan
        # Transformation Matrix (4x4 expected)
        matrix = None
        tm_index = block.find('Transformation Matrix:')
        if tm_index != -1:
            lines_after = block[tm_index:].splitlines()[1:]
            matrix_lines = [line.strip() for line in lines_after if line.strip() != ''][:4]
            try:
                matrix = np.array([list(map(float, line.split())) for line in matrix_lines])
            except Exception as e:
                print("Error parsing transformation matrix: {}".format(e))
                matrix = None
        record['TransformationMatrix'] = matrix
        records.append(record)

    df = pd.DataFrame(records)
    return df

def print_stats(df, folder_name):
    """
    Print descriptive statistics for selected metrics.
    Metrics: AlignTime, FilteredScanPoints, FitnessScore, NDTConverged, NDTReliability, NumIterations.
    """
    metrics = ['AlignTime', 'FilteredScanPoints', 'FitnessScore',
               'NDTConverged', 'NDTReliability', 'NumIterations']
    print("=== Statistics for folder: {} ===".format(folder_name))
    print(df[metrics].describe())
    print("")

def extract_voxel_size(folder_name):
    """
    Extract the voxel size from the folder name using the pattern "voxel_leaf_X".
    Returns the voxel size as a float, or None if not found.
    """
    match = re.search(r'px2_voxel_leaf_([0-9\.]+)', folder_name)
    if match:
        return float(match.group(1))
    else:
        return None

def main():
    parser = argparse.ArgumentParser(description="Iterate through log folders and aggregate NDT metrics.")
    parser.add_argument("root", type=str, help="Root directory to search (e.g., /disk2/logs/)")
    parser.add_argument("--folder", type=str, default=None, help="Specify a single folder to parse")
    parser.add_argument("--keyword", type=str, default=None, help="Only parse folders whose names contain this keyword")
    args = parser.parse_args()

    root_dir = args.root
    folders_to_parse = []

    if args.folder:
        folder_path = os.path.join(root_dir, args.folder)
        if os.path.isdir(folder_path):
            folders_to_parse.append(folder_path)
        else:
            print("Folder {} not found.".format(folder_path))
            return
    else:
        # Iterate through all subdirectories of root_dir
        for d in os.listdir(root_dir):
            full_path = os.path.join(root_dir, d)
            if os.path.isdir(full_path):
                if args.keyword and args.keyword not in d:
                    continue
                # Check if the expected log file exists in the folder
                log_file = os.path.join(full_path, "ndt_matching-11-stdout.log")
                if os.path.exists(log_file):
                    folders_to_parse.append(full_path)

    if not folders_to_parse:
        print("No folders found to parse.")
        return

    aggregated_data = []  # To store aggregated metrics for plotting

    for folder in folders_to_parse:
        log_file = os.path.join(folder, "ndt_matching-11-stdout.log")
        df = parse_log_file(log_file)
        if df.empty:
            continue
        print("Folder: {}".format(folder))
        print_stats(df, folder)
        # Compute aggregated metrics for this log file
        fitness_mean = df["FitnessScore"].mean()
        fitness_std = df["FitnessScore"].std()
        align_mean = df["AlignTime"].mean()
        align_std = df["AlignTime"].std()
        voxel_size = extract_voxel_size(os.path.basename(folder))
        if voxel_size is None:
            print("Warning: Could not extract voxel size from folder name: {}".format(folder))
            continue
        aggregated_data.append({
            'voxel_size': voxel_size,
            'fitness_mean': fitness_mean,
            'fitness_std': fitness_std,
            'align_mean': align_mean,
            'align_std': align_std
        })

    if not aggregated_data:
        print("No aggregated data to plot.")
        return

    # Sort aggregated data by voxel size
    aggregated_data = sorted(aggregated_data, key=lambda x: x['voxel_size'])
    agg_df = pd.DataFrame(aggregated_data)

    # Build a title with the input arguments information using .format()
    title_info = "Data of: {}".format(args.root)
    if args.folder:
        title_info += " | Folder: {}".format(args.folder)
    if args.keyword:
        title_info += " | Keyword: {}".format(args.keyword)

    # Plot aggregated metrics: two subplots (Fitness Score and Align Time) vs. Voxel Size
    plt.style.use('seaborn-darkgrid')
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Fitness Score plot
    axs[0].errorbar(agg_df["voxel_size"], agg_df["fitness_mean"], yerr=agg_df["fitness_std"],
                    fmt='o', capsize=5, markersize=5, linewidth=2)
    axs[0].set_ylabel("Fitness Score")
    axs[0].set_title("Aggregated Fitness Score")
    axs[0].grid(True)

    # Align Time plot
    axs[1].errorbar(agg_df["voxel_size"], agg_df["align_mean"], yerr=agg_df["align_std"],
                    fmt='o', capsize=5, markersize=5, linewidth=2)
    axs[1].set_ylabel("Align Time (ms)")
    axs[1].set_xlabel("Voxel Size")
    axs[1].set_title("Aggregated Align Time")
    axs[1].grid(True)

    # Add a super title with parsed arguments information
    plt.suptitle("Aggregated Metrics\n" + title_info, fontsize=14)
    plt.subplots_adjust(top=0.88)  # Adjust top margin so suptitle fits inside the figure

    plt.tight_layout(rect=[0, 0, 1, 0.85])
    plt.show()

if __name__ == "__main__":
    main()

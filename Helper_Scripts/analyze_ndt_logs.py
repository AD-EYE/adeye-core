#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###################################################################3###
## Script that helps interpret the logs of localization functionality.
########################################################################
"""
localizer logs are to be found in /.log folder, named "ndt_matching-13-stdout.log"
press Ctrl+H to show the hidden folers.

Usage example:
    python analyze_ndt_logs.py /PATH_TO_YOUR_LOG/ndt_matching-13-stdout.log
To compare 2 logfiles:
    python analyze_ndt_logs.py /PATH_1/ndt_matching-13-stdout.log --log2 /PATH_2/ndt_matching-13-stdout.log
"""


import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse

# ------------------------------
# Parsing Function
# ------------------------------
def parse_log_file(file_path):
    """
    Parse the log file and extract key fields from each block.
    """
    try:
        import io
        with io.open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print("Error opening file {}: {}".format(file_path, e))
        return pd.DataFrame()

    # Split file content into blocks based on dash lines
    blocks = re.split(r'-{5,}', content)
    records = []
    for block in blocks:
        if 'Sequence:' not in block:
            continue  # Skip blocks without valid record info

        record = {}
        m = re.search(r'Timestamp:\s*([\d\.]+)', block)
        record['Timestamp'] = float(m.group(1)) if m else np.nan

        m = re.search(r'Fitness Score:\s*([\d\.]+)', block)
        record['FitnessScore'] = float(m.group(1)) if m else np.nan

        m = re.search(r'Number of Filtered Scan Points:\s*(\d+)', block)
        record['FilteredScanPoints'] = int(m.group(1)) if m else np.nan

        m = re.search(r'NDT has converged:\s*(\d+)', block)
        record['NDTConverged'] = int(m.group(1)) if m else np.nan

        m = re.search(r'Number of Iterations:\s*(\d+)', block)
        record['NumIterations'] = int(m.group(1)) if m else np.nan

        m = re.search(r'NDT Reliability:\s*([\d\.]+)', block)
        record['NDTReliability'] = float(m.group(1)) if m else np.nan

        m = re.search(r'Align time:\s*([\d\.]+)', block)
        record['AlignTime'] = float(m.group(1)) if m else np.nan

        # Extract Transformation Matrix (4x4 expected)
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

# ------------------------------
# Descriptive Statistics Printer
# ------------------------------
def print_stats(df, label):
    """
    Print descriptive statistics for selected metrics.
    """
    metrics = ['AlignTime', 'FilteredScanPoints', 'FitnessScore',
               'NDTConverged', 'NDTReliability', 'NumIterations']
    print("=== Statistics for {} ===".format(label))
    print(df[metrics].describe())
    print("")

# ------------------------------
# Plotting Function
# ------------------------------
def plot_comparison(df1, df2=None):
    """
    Generate one figure with subplots for:
      - Processing Time (AlignTime)
      - Fitness Score
    For each subplot, if a second file is provided, both data files are overlaid.
    """
    # Use a nice style for better aesthetics
    plt.style.use('seaborn-darkgrid')

    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot Processing Time (AlignTime)
    axs[0].plot(df1["Timestamp"], df1["AlignTime"], marker="o", markersize=8, linestyle="--", linewidth=2, label="File 1 AlignTime")
    if df2 is not None:
        axs[0].plot(df2["Timestamp"], df2["AlignTime"], marker="s", markersize=8, linestyle="--", linewidth=2, label="File 2 AlignTime")
    axs[0].set_ylabel("AlignTime (ms)", fontsize=12)
    axs[0].set_title("Processing Time Comparison", fontsize=14)
    axs[0].legend(fontsize=12)
    axs[0].grid(True)

    # Plot Fitness Score
    axs[1].plot(df1["Timestamp"], df1["FitnessScore"], marker="o", markersize=8, linestyle="-", linewidth=2, label="File 1 FitnessScore")
    if df2 is not None:
        axs[1].plot(df2["Timestamp"], df2["FitnessScore"], marker="s", markersize=8, linestyle="-", linewidth=2, label="File 2 FitnessScore")
    axs[1].set_ylabel("Fitness Score", fontsize=12)
    axs[1].set_title("Fitness Score Comparison", fontsize=14)
    axs[1].set_xlabel("Timestamp", fontsize=12)
    axs[1].legend(fontsize=12)
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()




def main():
    parser = argparse.ArgumentParser(
        description="NDT Log File Analysis: Compare localization performance."
    )
    parser.add_argument("log1", type=str, help="Path to the first log file")
    parser.add_argument("--log2", type=str, default=None, help="Path to the second log file (optional)")
    args = parser.parse_args()

    # Parse the first log file
    df1 = parse_log_file(args.log1)
    if df1.empty:
        print("Error: The first log file did not contain valid records.")
        return

    # Print statistics for file 1
    print_stats(df1, "File 1")

    df2 = None
    if args.log2:
        df2 = parse_log_file(args.log2)
        if df2.empty:
            print("Error: The second log file did not contain valid records.")
            return
        # Print statistics for file 2
        print_stats(df2, "File 2")

    # Plot comparisons: Processing Time and Fitness Score
    plot_comparison(df1, df2)

if __name__ == "__main__":
    main()

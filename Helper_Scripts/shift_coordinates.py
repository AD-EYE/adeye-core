import pandas as pd
import sys
import os
import math
import argparse


# ADEYE Team 2025 - Moving the vector Map in space to align it with the point cloud map
#############################################################################################################################
# This script reads the CSV file containing point coordinates, applies a translation and rotation transformation,
# and saves the transformed coordinates to a new CSV file.
#############################################################################################################################

def shift_and_rotate_coordinates(input_file, dx, dy, angle_deg=0):
    # Read the first line of the input file
    with open(input_file, 'r') as f:
        first_line = f.readline()

    # First line should be a header or comment, so we keep it as is

    # Read the rest of the CSV file starting from the second line
    df = pd.read_csv(input_file, header=None, skiprows=1)

    # Check that at least columns 4 and 5 (index 4 and 5) exist
    if df.shape[1] <= 5:
        print("Error: Expected at least 6 columns in the CSV file.")
        return

    # Convert angle from degrees to radians
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    # Extract original X and Y
    x = df[4]
    y = df[5]

    # Apply rotation
    x_rot = x * cos_a - y * sin_a
    y_rot = x * sin_a + y * cos_a

    # Apply translation
    df[4] = x_rot + dx
    df[5] = y_rot + dy

    # Generate output filename
    base, ext = os.path.splitext(input_file)
    output_file = f"{base}_rotated_shifted{ext}"

    # Save modified DataFrame starting from the second line
    with open(output_file, 'w') as f:
        f.write(first_line)  # Write the first line
        df.to_csv(f, index=False, header=False, mode='a')

    print(f"Rotated and shifted coordinates saved to: {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Shift and rotate coordinates in a CSV file.")
    parser.add_argument("input_file", help="Path to the input CSV file")
    parser.add_argument("--dx", type=float, default=0.0, help="Shift in X direction")
    parser.add_argument("--dy", type=float, default=0.0, help="Shift in Y direction")
    parser.add_argument("--angle_deg", type=float, default=0.0, help="Rotation angle in degrees")

    args = parser.parse_args()

    shift_and_rotate_coordinates(args.input_file, args.dx, args.dy, args.angle_deg)

#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import ast

# Path to your input CSV file
input_csv_file = '2024-10-22-05-31-48.csv'  # Replace with your actual input file path
output_csv_file = 'output_file.csv'      # Path to save the updated CSV

# Read the CSV file into a pandas DataFrame
df = pd.read_csv(input_csv_file)

# Print the column names to identify twist_x and twist_y columns
print("Columns in CSV:", df.columns)

# Specify the columns for twist x and y
twist_x_column = '/vicon/BEAST_02/odom_twist_twist_linear_x'  # Replace with the actual column name for twist x
twist_y_column = '/vicon/BEAST_02/odom_twist_twist_linear_y'  # Replace with the actual column name for twist y

joy_column = '/joy_axes'

data_series = df[joy_column].apply(ast.literal_eval)

y_values = data_series.apply(lambda x: x[7]) 

# Check if the specified columns exist in the DataFrame
if twist_x_column in df.columns and twist_y_column in df.columns:
    # Calculate the velocity magnitude
    df['velocity_magnitude'] = np.sqrt(df[twist_x_column]**2 + df[twist_y_column]**2)
    
    # Print the updated DataFrame
    print(df[[twist_x_column, twist_y_column, 'velocity_magnitude']].head())

    # Save the updated DataFrame back to a CSV file
    df.to_csv(output_csv_file, index=False)
    print(f"Updated CSV saved to {output_csv_file}")
else:
    print(f"Columns '{twist_x_column}' or '{twist_y_column}' not found in the DataFrame.")

# Check if the specified columns exist in the DataFrame
ii = 0
if joy_column in df.columns:
    # Calculate the velocity magnitude
    df['joy_rz'] = y_values
    
    # Print the updated DataFrame
    print(df[[joy_column, 'joy_rz']].head())

    # Save the updated DataFrame back to a CSV file
    df.to_csv(output_csv_file, index=False)
    print(f"Updated CSV saved to {output_csv_file}")
else:
    print(f"Columns '{joy_column}' not found in the DataFrame.")
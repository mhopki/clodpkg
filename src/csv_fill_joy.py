#!/usr/bin/env python3

import rospy
import pandas as pd

# Read the CSV file into a pandas DataFrame
csv_file = '2024-10-22-05-40-09.csv'  # Replace with your CSV file path
df = pd.read_csv(csv_file)

# Specify the column you want to check (replace 'your_column_name' with the actual column name)
columns = ['/joy_axes', '/vicon/BEAST_02/odom_twist_twist_linear_x', '/vicon/BEAST_02/odom_twist_twist_linear_y', '/vicon/BEAST_02/odom_twist_twist_linear_z', '/vicon/BEAST_02/odom_pose_pose_position_x', '/vicon/BEAST_02/odom_pose_pose_position_y']
for column_name in columns:

    # Use a copy of the DataFrame to avoid warnings
    df[column_name] = df[column_name].copy()

    # Iterate through the specified column
    for i in range(1, len(df)):
        # Check if the current item is NaN and the previous item is not NaN
        if pd.isna(df.loc[i, column_name]) and not pd.isna(df.loc[i - 1, column_name]):
            # Set the current item to the previous item
            df.loc[i, column_name] = df.loc[i - 1, column_name]

    # Optionally, you can save the modified DataFrame back to a new CSV
    df.to_csv(csv_file, index=False)  # Replace with desired output file path

    # Print the modified DataFrame
    print(df)

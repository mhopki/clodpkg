#!/usr/bin/env python3

import rospy
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import ast

# Read the CSV file into a pandas DataFrame
csv_file = 'output_file.csv'  # Path to your CSV file
df = pd.read_csv(csv_file)

# Print the column names to identify the ones you want to use
print(df.columns)

# Choose the column you want to plot (for example, a specific topic field)
# Replace 'your_column_name' with the actual column name from your CSV
timestamp_column = 'timestamp'  # Assuming 'timestamp' is the column with time values
data_column = '/joy_axes'  # Replace this with the actual column name you want to plot
data_column2 = '/vicon/BEAST_02/odom_pose_pose_position_x' #'/vicon/BEAST_02/odom_twist_twist_linear_x'  # Replace this with the actual column name you want to plot
data_column3 = '/vicon/BEAST_02/odom_pose_pose_position_y'
data_column4 = '/vicon/BEAST_02/odom_twist_twist_linear_x'
data_column5 = '/vicon/BEAST_02/odom_twist_twist_linear_y'
data_column6 = '/vicon/BEAST_02/odom_twist_twist_linear_z'
data_column7 = 'velocity_magnitude'
data_column8 = 'joy_rz'
print(df[data_column][0])
print(np.array(df[data_column][0]))
print(list(ast.literal_eval(df[data_column][902]))[7])

data_series = df[data_column].apply(ast.literal_eval)

y_values = data_series.apply(lambda x: x[7]) 

start = 3000#0#3000
end = 19363#3000#19363#17177
chops = [2220,3000, 3000, 4180]

#################################
# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(df[timestamp_column][start:end], y_values[start:end], label=data_column)
#plt.plot(df[timestamp_column][:2550], df[data_column2][:2550], label=data_column2)

# Plot the data
#plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column], df[data_column][:][7], label=data_column)

# Add labels and title
plt.xlabel('Timestamp (seconds)')
plt.ylabel('Value')
plt.title(f'{data_column} vs Time' + csv_file)
plt.ylim(bottom=0, top=1)

# Add a legend
plt.legend()

#################################
# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(df[timestamp_column][start:end], df[data_column8][start:end], label=data_column8)
#plt.plot(df[timestamp_column][:2550], df[data_column2][:2550], label=data_column2)

# Plot the data
#plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column], df[data_column][:][7], label=data_column)

# Add labels and title
plt.xlabel('Timestamp (seconds)')
plt.ylabel('Value')
plt.title(f'{data_column8} vs Time' + csv_file)
plt.ylim(bottom=0, top=1)

# Add a legend
plt.legend()

#################################
plt.figure(figsize=(10, 6))
plt.plot(df[timestamp_column][start:end], df[data_column4][start:end], label=data_column4)
#plt.plot(df[timestamp_column][:2550], df[data_column2][:2550], label=data_column2)

# Plot the data
#plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column], df[data_column][:][7], label=data_column)

# Add labels and title
plt.xlabel('Timestamp (seconds)')
plt.ylabel('Value')
plt.title(f'{data_column4} vs Time' + csv_file)
#plt.xlim(left=0, right=5)  # Set the x-axis limits (replace with your desired values)
plt.ylim(bottom=-5, top=5)

# Add a legend
plt.legend()

###############################
plt.figure(figsize=(10, 6))
plt.plot(df[timestamp_column][start:end], df[data_column5][start:end], label=data_column5)
#plt.plot(df[timestamp_column][:2550], df[data_column2][:2550], label=data_column2)

# Plot the data
#plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column], df[data_column][:][7], label=data_column)

# Add labels and title
plt.xlabel('Timestamp (seconds)')
plt.ylabel('Value')
plt.title(f'{data_column5} vs Time' + csv_file)
plt.ylim(bottom=-5, top=5)

# Add a legend
plt.legend()

###############################
plt.figure(figsize=(10, 6))
plt.plot(df[timestamp_column][start:end], df[data_column7][start:end], label=data_column7)
#plt.plot(df[timestamp_column][:2550], df[data_column2][:2550], label=data_column2)

# Plot the data
#plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column], df[data_column][:][7], label=data_column)

# Add labels and title
plt.xlabel('Timestamp (seconds)')
plt.ylabel('Value')
plt.title('Velocity vs Time' + csv_file)
plt.ylim(bottom=-5, top=5)

# Add a legend
plt.legend()

###############################################
# Plot the data
plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column][:2550], df[data_column][:2550], label=data_column)
plt.plot(df[data_column2][start:end], df[data_column3][start:end], label=data_column2)

# Plot the data
#plt.figure(figsize=(10, 6))
#plt.plot(df[timestamp_column], df[data_column][:][7], label=data_column)

# Add labels and title
plt.xlabel('Position X (m)')
plt.ylabel('Position Y (m)')
plt.title('X vs Y '  + csv_file)

# Add a legend
plt.legend()

# Show the plot
plt.show()

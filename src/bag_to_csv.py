#!/usr/bin/env python3

import rospy
import rosbag
import csv
import os
from collections import defaultdict

def flatten_msg(msg, parent_key=''):
    """Flatten a ROS message into a dictionary."""
    items = []
    for field in msg.__slots__:
        value = getattr(msg, field)
        new_key = f"{parent_key}_{field}" if parent_key else field
        if hasattr(value, '__slots__'):  # If the field is another message (complex type), recurse
            items.extend(flatten_msg(value, new_key).items())
        else:  # If it's a simple type, just add it
            items.append((new_key, value))
    return dict(items)

def bag_to_csv(bag_file, output_file):
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Dictionary to store data based on timestamp
    data_dict = defaultdict(dict)
    
    # Create a set to keep track of all message fields from all topics
    fields = set()

    # Iterate through each message in the bag
    for topic, msg, t in bag.read_messages():
        timestamp = t.to_sec()

        # Flatten the message fields (handles nested fields)
        field_values = flatten_msg(msg)
        fields.update([f"{topic}_{field}" for field in field_values.keys()])

        # Store the flattened message data at the given timestamp
        for field, value in field_values.items():
            data_dict[timestamp][f"{topic}_{field}"] = value
    
    # Close the bag file
    bag.close()

    # Sort timestamps
    sorted_timestamps = sorted(data_dict.keys())

    # Write data to a CSV file
    with open(output_file, 'w') as f:
        writer = csv.writer(f)

        # Write header
        header = ['timestamp'] + sorted(fields)
        writer.writerow(header)

        # Write rows for each timestamp
        for timestamp in sorted_timestamps:
            row = [timestamp] + [data_dict[timestamp].get(field, '') for field in sorted(fields)]
            writer.writerow(row)

if __name__ == '__main__':
    bag_file = '2024-10-22-05-40-09.bag'  # Path to your ROS bag
    output_dir = '/home/malakhi/clod_pid_bags/'+bag_file[0:-4]+".csv"        # Directory to save CSV files
    bag_to_csv(bag_file, output_dir)

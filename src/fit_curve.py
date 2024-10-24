#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load your data from a CSV file
csv_file = 'output_file.csv'  # Replace with your actual file path
df = pd.read_csv(csv_file)

# Assume 'motor_command' is your independent variable and 'velocity' is your dependent variable

slice_1 = df['joy_rz'][2220:3000]  # Replace with your actual column name
slice_2 = df['joy_rz'][3000:4180]  # Replace with your actual column name
motor_commands = pd.concat([slice_1, slice_2], ignore_index=True)

slice_1 = df['velocity_magnitude'][2220:3000]             # Replace with your actual column name
slice_2 = df['velocity_magnitude'][3000:4180]             # Replace with your actual column name
velocity = pd.concat([slice_1, slice_2], ignore_index=True)

non_zero_indices = df.index[df['joy_rz'] != 0].tolist()
motor_commands = df['joy_rz'].iloc[non_zero_indices]
velocity = df['velocity_magnitude'].iloc[non_zero_indices]

# Fit a polynomial of degree n (e.g., 2 for a quadratic)
degree = 3  # Change this to fit different degrees
coefficients = np.polyfit(motor_commands, velocity, degree)

# Create a polynomial function from the coefficients
polynomial = np.poly1d(coefficients)

# Generate values for plotting the polynomial
x_range = np.linspace(motor_commands.min(), motor_commands.max(), 100)
fitted_values = polynomial(x_range)

# Create a modified polynomial that lowers the output at 0
# Here we're simply modifying the constant term (the last coefficient)
modified_coefficients = coefficients.copy()
modified_coefficients[-4] = -13.02367641 + 9.2#-13.02367641
modified_coefficients[-3] = 17.82966064 - 11#17.82966064 
modified_coefficients[-2] = 0  # Set the constant term to 0 for the modified polynomial
modified_coefficients[-1] = 0
modified_polynomial = np.poly1d(modified_coefficients)

# Generate values for plotting the modified polynomial
modified_fitted_values = modified_polynomial(x_range)

# Create a polynomial function from the coefficients
polynomial = np.poly1d(coefficients)

# Evaluate the polynomial at the motor commands to predict velocities
predicted_velocities = modified_polynomial(motor_commands)

# Plot the original data and the fitted polynomial
plt.scatter(motor_commands, velocity, label='Data', color='blue', alpha=0.5)
plt.plot(x_range, fitted_values, label=f'Polynomial Fit (degree {degree})', color='red')
plt.plot(x_range, modified_fitted_values, label='Modified Polynomial Fit', color='green')
#plt.plot(motor_commands, predicted_velocities, label=f'Predicted Velocity (Polynomial Fit, degree {degree})', color='yellow')
plt.xlabel('Motor Commands')
plt.ylabel('Velocity')
plt.ylim(bottom=0, top=5)
plt.title('Motor Commands vs. Velocity with Polynomial Fit')
plt.legend()
plt.grid()
plt.show()

# Print the polynomial coefficients
print(f"Polynomial coefficients (highest to lowest degree): {coefficients}")

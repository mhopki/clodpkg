import numpy as np 
from numpy import genfromtxt
import matplotlib.pyplot as plt 
import sys


def read_from_csv(file_name) : 
	my_data = genfromtxt(file_name, delimiter=',')
	my_data = my_data[1:]
	times = my_data[:,0]
	positions = my_data[:,1:3]
	velocities = my_data[:,3:5]
	return times, positions, velocities



def read_from_csv_min_dist_difference(file_name, min_dist) :
	times, positions, velocities = read_from_csv(file_name)
	
	sparse_positions = []
	sparse_velocities = []
	sparse_times = [] 
	
	i = 0
	sparse_positions.append(positions[0])
	sparse_velocities.append(velocities[0])
	sparse_times.append(times[0])
	
	dist_increment = 0
	last_added_i = 0
	for i in range(1, len(times)):
		dist_increment += np.linalg.norm(positions[i] - positions[i-1])
		if dist_increment >= min_dist :
			sparse_positions.append(positions[i])
			sparse_velocities.append(velocities[i])
			sparse_times.append(times[i])
			dist_increment = 0 
			last_added_i = i 
	
	if last_added_i < len(times)-1:
		sparse_positions.append(positions[len(times)-1])
		sparse_velocities.append(velocities[len(times)-1])
		sparse_times.append(times[len(times)-1])
	
	sparse_positions = np.array(sparse_positions)
	sparse_velocities = np.array(sparse_velocities)
	sparse_times = np.array(sparse_times)
	
	return sparse_times, sparse_positions, sparse_velocities
	
	
	
def read_from_csv_min_time_difference(file_name, min_time) :
	times, positions, velocities = read_from_csv(file_name)
	
	sparse_positions = []
	sparse_velocities = []
	sparse_times = [] 
	
	i = 0
	sparse_positions.append(positions[0])
	sparse_velocities.append(velocities[0])
	sparse_times.append(times[0])
	
	time_increment = 0
	last_added_i = 0
	for i in range(1, len(times)):
		time_increment += times[i] - times[i-1]
		if time_increment >= min_time :
			sparse_positions.append(positions[i])
			sparse_velocities.append(velocities[i])
			sparse_times.append(times[i])
			time_increment = 0 
			last_added_i = i 
	
	if last_added_i < len(times)-1:
		sparse_positions.append(positions[len(times)-1])
		sparse_velocities.append(velocities[len(times)-1])
		sparse_times.append(times[len(times)-1])
	
	sparse_positions = np.array(sparse_positions)
	sparse_velocities = np.array(sparse_velocities)
	sparse_times = np.array(sparse_times)
	
	return sparse_times, sparse_positions, sparse_velocities	
	
	
	


if __name__ == "__main__" :
	file_name = 'traj_1.csv'
	times, positions, velocities = read_from_csv(file_name)
	#print(positions[:10])
	
	plt.figure()
	plt.scatter(positions[:,0], positions[:,1])
	plt.show()
	
	sys.exit()
	
	
	sparse_times, sparse_positions, sparse_velocities = read_from_csv_min_dist_difference(file_name, 0.005)

	sparse_times, sparse_positions, sparse_velocities = read_from_csv_min_time_difference(file_name, 0.1)
	
	


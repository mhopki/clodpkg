import numpy as np 
from numpy import genfromtxt
import matplotlib.pyplot as plt 
import sys


def set_init_quaternion_from_csv(file_name) : 
	my_data = genfromtxt(file_name, delimiter=',')
	my_data = my_data[1:]

	first_pos = my_data[1, 1:3]
	
	first_vel = my_data[1, 3:5]
	first_vel = first_vel / np.linalg.norm(first_vel)
	
	xaxis = first_vel 
	angle = np.arctan2(-xaxis[0], xaxis[1])#xaxis[1], xaxis[0])
	quaternion = np.zeros(4)
	quaternion[2] = np.sin(angle * 0.5)
	quaternion[3] = np.cos(angle * 0.5)
	
	return first_pos, quaternion
	

if __name__ == "__main__" :
	file_name = 'ag_3_fin_v_1_a_1_traj_2.csv'
	pos, quat = set_init_quaternion_from_csv(file_name)
	print(pos, quat)

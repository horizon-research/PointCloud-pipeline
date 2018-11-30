'''
	Plot:
		frame-wise translational and rotational error 
'''

import matplotlib.pyplot as plt
import numpy as np
import sys
import time 
import math

def load_and_transform(filename):
	'''
	Load KITTI format ground-truth or pose estimation results. 
	Each line in the file represents a 3 * 4 matrix. 

	Returns:
		A list of numpy matrix
	'''
	with open(filename) as f:
	    poses = f.readlines()

	poses = [x.strip().split(' ') for x in poses[:]]
	poses_matrix = list()

	for mt in poses[:]:
		'''
			Transform 3 * 4 into 4 * 4 (needed when calculating the inverse matrix).
		'''
		list_ = [float(i) for i in mt]

		list_.append(.0)
		list_.append(.0)
		list_.append(.0)
		list_.append(1.0)

		matrix_calcaulated = np.matrix(list_).reshape(4, 4)
		poses_matrix.append(matrix_calcaulated)
	
	return poses_matrix

def load_ests(filename):
	'''
	Load ICP iteration results. (In this file stores the result of each icp iteration, instead of only the final result).  
		Example:
		3 * 4 (iter1, frame1-0)
		3 * 4 (iter2, frame1-0)
		3 * 4 (iter3, frame1-0)
		...
		3 * 4 (iter40, frame1-0)
		3 * 4 (iter1, frame2-1)
		3 * 4 (iter2, frame2-1)
		3 * 4 (iter3, frame2-1)
		...
		3 * 4 (iter40, frame2-1)

	Returns:
		A list of numpy matrix list, each stores the matrix results after each iteration for the frame pairs.
		Example: 
		[[m1, m2, ... m40],
		 [m1, m2, ... m40],
		 ...
		]
	'''
	with open(filename) as f:
	    poses = f.readlines()

	total_est=list() # stores a list of icp result lists, i.e., frame_est
	frame_est=list() # stores a list of icp results (tranformation matrix given by each iteration)

	for l_ in poses:
		if l_ == '\n':
			total_est.append(frame_est.copy())
			frame_est=list()
		elif l_ == '-\n':
			pass
		else:
			frame_est.append(l_.strip().split(' '))

	''' transform list into numpy matrix
	'''
	total_est_matrix=list()
	for est_ in total_est:
		# est_: all the results for each pairwise registration
		frame_est_matrix=list()
		for mt in est_:
			# mt: result of each icp iteration 
			list_ = [float(i) for i in mt]
			list_.append(.0)
			list_.append(.0)
			list_.append(.0)
			list_.append(1.0)

			matrix_calcaulated = np.matrix(list_).reshape(4, 4)
			frame_est_matrix.append(matrix_calcaulated)

		total_est_matrix.append(frame_est_matrix)

	print(len(total_est))
	print(len(total_est_matrix))

	return total_est_matrix

def estimate_error(matrix_list_gt, total_est_matrix, pose_ransac_matrix):
	'''
	'''
	dist = list()
	dist.append(.0)

	tranlational_err_list = list()
	rotational_err_list = list()

	'''
		Ground Truth
	'''
	for i, m in enumerate(matrix_list_gt[:]):
		''' Calculate distance
		'''
		if i > 0:
			# the first line of gt is a identity matrix (of no use)
			dx = m[2, 3] - matrix_list_gt[i-1][2,3]
			dy = m[0, 3] - matrix_list_gt[i-1][0,3]
			dz = m[1, 3] - matrix_list_gt[i-1][1,3]

			dist_delta = math.sqrt(dx*dx+dy*dy+dz*dz)

			dist.append(dist[i-1] + dist_delta)

	'''
		Estimated poses
	'''
	translational_error_total = list()
	rotational_error_total = list()

	for i0, frame_mts in enumerate(total_est_matrix[:]):
		''' frames
		'''

		# i0 == 1: 0 <- 1 registration result

		translational_error_frame = list()
		rotational_error_frame = list()

		for i, m in enumerate(frame_mts):
			# i + 1: iteration number
			''' 0 - 39 estimtated matrix
			'''
			''' Calculate rotational and translational error
			'''
			if i0 > 0:

				pose_delta_gt = np.matmul(matrix_list_gt[i0 - 1].I, matrix_list_gt[i0])
				pose_delta_est = np.matmul(total_est_matrix[i0 - 1][i], pose_ransac_matrix[i0 - 1])
				
				if i == len(frame_mts) - 1:
					print("Frame: ", i0 - 1, "<---", i0)
					print("delta gt\n", pose_delta_gt)
					print("delta est\n", pose_delta_est)
					print('\n\n')

				pose_error = np.matmul(pose_delta_est.I, pose_delta_gt)

				translational_error = math.sqrt( \
					pose_error[0, 3] * pose_error[0, 3] + \
					pose_error[1, 3] * pose_error[1, 3] + \
					pose_error[2, 3] * pose_error[2, 3])

				translational_error_frame.append(translational_error)

				a = pose_error[0, 0]
				b = pose_error[1, 1]
				c = pose_error[2, 2]
				d = 0.5 * (a + b + c - 1.0)

				rotational_error = math.acos(max(min(d, 1.0), -1.0))
				rotational_error_frame.append(rotational_error)

		if i0 > 0:
			# ith: i ----> previous
			translational_error_total.append(translational_error_frame)
			rotational_error_total.append(rotational_error_frame)

	''' Driving distance
	'''
	return dist, translational_error_total, rotational_error_total

'''
	Load pose ground truth && Transform to matrix
'''
fname1="../Kitti/Ground-Truth/dataset/poses/00.txt"
pose_gt_matrix = load_and_transform(fname1)

# ransac results
fransac='./result1/ransac_delta.txt'
pose_ransac_matrix = load_and_transform(fransac)

# icp deltas
fname2='./icp_results.txt'
total_est_matrix = load_ests(fname2)
dist, translational_error_total, rotational_error_total = estimate_error(pose_gt_matrix, total_est_matrix, pose_ransac_matrix)

'''
	plot 
'''

fig=plt.figure()

trans_err = fig.add_subplot(3,2,1)
rot_err = fig.add_subplot(3,2,2)

trans_err.set_title("Translational Error (Absolute)")
rot_err.set_title("Rotational Error (Absolute)")

for f_ in [25,50,75,105,120]:
	trans_err.plot(range(40), translational_error_total[f_], '*-', label='#' + str(f_))
	rot_err.plot(range(40), rotational_error_total[f_], '^-', label='#' + str(f_))
trans_err.legend(loc='upper right')
rot_err.legend(loc='upper right')

plt.show()

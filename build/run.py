#!/home/nile/anaconda3/bin/python
import os

program = "./pc-registration"

use_keypoint = str(0)

normal_R = str(0.45)
FPFH_R = str(0.55)
ransac_threshold = str(0.20)
icp_transEps = str(1e-9)
icp_EuclFitEps = str(1e-6)
corrs_d=1.20
out_mul=100
icp_corresDist = str(float(corrs_d))
icp_outlThresh = str(float(corrs_d * out_mul))

dataset = "../Kitti/Data/00/velodyne/"

kernel_keyPoints = str('NARF')
kernel_descriptors = str('SHOT')
kernel_corrEst = str('corrEst')
kernel_corrRej = str('corrRej')

flag_reciprocal = str('True')

icp_solver = str("SVD")
icp_flag_ransac=str("True")
icp_flag_reciprocal= str("True")

### mkdir 
max_dir_num=0
for i in os.listdir("../"):
	# print(i)
	if "result" in i:
		if max_dir_num < int(i[6:]):
			max_dir_num = int(i[6:])
result_dir_num = max_dir_num + 1
result_dir = "../result" + str(result_dir_num) + "/"

# mkdir && create files
if not os.path.exists(result_dir):
	print("mkdir " + result_dir)
	os.mkdir(result_dir)

result_file = result_dir + "pose_result_kitti.txt"
icp_delta_file = result_dir + "icp_delta.txt"
ransac_delta_file = result_dir + "ransac_delta.txt"
stage_time_file = result_dir + "stage_time.txt"
kdtree_time_file = result_dir + "kdtree_time.txt"

### mkdir ends

cmd = program + ' ' + normal_R + ' ' + FPFH_R + ' ' + \
ransac_threshold + ' ' + icp_transEps + ' ' + icp_corresDist + ' ' + \
icp_EuclFitEps + ' ' + icp_outlThresh + ' ' + result_file + ' ' + stage_time_file + \
' ' + dataset + ' ' + ransac_delta_file + ' ' + icp_delta_file + ' ' + use_keypoint + \
' ' + kernel_keyPoints + ' ' + kernel_descriptors + ' ' + kernel_corrEst + ' ' + kernel_corrRej + \
' ' + flag_reciprocal + ' '+ icp_solver + ' ' + icp_flag_ransac + ' ' + icp_flag_reciprocal + \
' ' + kdtree_time_file

f = open(result_dir + "params.txt", "w+")
f.write(cmd)
f.close()

print(cmd)
os.system(cmd)

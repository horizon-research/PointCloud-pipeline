import matplotlib.pyplot as plt
import sys
import time 
import math

'''
	PLOT:
		trajectory 
		error in 3 axes
'''

# path to the ground truth
fname="../../Kitti/Ground-Truth/dataset/poses/00.txt"
# results
#gname='../pc-pipeline-lab/result' + sys.argv[1] + '/pose_result_kitti.txt'

fname="../../Kitti/Ground-Truth/dataset/poses/00.txt"
#gname="./pc-code-examination-lab/result" + sys.argv[1] + "/pose_result_kitti.txt"

gname="./result" + sys.argv[1] + "/pose_result_kitti.txt"
sname="./result" + sys.argv[1] + "/params.txt"

with open(sname) as f:
    specs = f.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
specs = [x.strip() for x in specs] 
print(specs)

fig=plt.figure()
# # gt_xyz = fig.add_subplot(2,1,1)
# # ours_xyz = fig.add_subplot(2,1,2)
trajectory = fig.add_subplot(2,2,1)
z_com = fig.add_subplot(2,2,2)
y_com = fig.add_subplot(2,2,3)
x_com = fig.add_subplot(2,2,4)

x=[]
y=[]
z=[]
d=[]
err=[]

with open(fname) as f:
    gt_content = f.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
gt_content = [x.strip() for x in gt_content] 

with open(gname) as f:
    ours_content2 = f.readlines()
ours_content2 = [x.strip() for x in ours_content2]

n=len(ours_content2)
print(n)
if len(sys.argv) == 3:
	n = int(sys.argv[2])

#groudtruth
for i, v in enumerate(gt_content[:n]):
	# print(i.split(' ')[3], i.split(' ')[7], i.split(' ')[11])
	d.append(i)
	# refer to the gt result files
	x_gt = float(v.split(' ')[3])
	y_gt = float(v.split(' ')[7])
	z_gt = float(v.split(' ')[11])

	x.append(x_gt)
	y.append(y_gt)
	z.append(z_gt)

x1=[]
y1=[]
z1=[]
d1 =[]

#ours
for i, v in enumerate(ours_content2[:n]):
	d1.append(i)
	x_ = float(v.split(' ')[3])
	y_ = float(v.split(' ')[7])
	z_ = float(v.split(' ')[11])

	x1.append(x_)
	y1.append(y_)
	z1.append(z_)

# ours_xyz.clear()
# gt_xyz.clear()
# trajectory.clear()

# plt.subplot(221)
# gt_xyz.set_title('Position(xyz) - Ground Truth')
# gt_xyz.axis([0, n + 100, -200, 600])

# gt_xyz.plot(d, x, 'r', label='x')
# gt_xyz.plot(d, y, 'g', label='y')
# gt_xyz.plot(d, z,'b', label='z')
# gt_xyz.legend(bbox_to_anchor=(1.06, 0.60))

# ours_xyz.set_title('Position(xyz) - Estimated Result')
# ours_xyz.axis([0, n + 100, -200, 600])
# ours_xyz.plot(d1, x1, 'r', label='x')
# ours_xyz.plot(d1, y1, 'g', label='y')
# ours_xyz.plot(d1, z1,'b', label='z')
# ours_xyz.legend(bbox_to_anchor=(1.06, 0.60))

trajectory.set_title("Trajectory")
trajectory.plot(z, x, 'r--', label='GT')
trajectory.plot(z1, x1, 'g--', label='EST')
# trajectory.legend(loc='upper right')
# trajectory.legend(bbox_to_anchor=(1.08, 0.60))
trajectory.legend()


z_com.set_title("Z value comparison")
z_com.plot(d, z, 'r', label='z - GT')
z_com.plot(d1, z1, 'g', label='z - EST')
z_com.legend()

y_com.set_title("Y value comparison")
y_com.plot(d, y, 'r', label='y - GT')
y_com.plot(d1, y1, 'g', label='y - EST')
y_com.legend()

x_com.set_title("X value comparison")
x_com.plot(d, x, 'r', label='x - GT')
x_com.plot(d1, x1, 'g', label='x - EST')
x_com.legend()

plt.show()
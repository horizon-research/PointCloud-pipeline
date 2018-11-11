### Point Cloud Registration
Pairwise registration of point cloud data using PCL. 

-------------------------------
#### Overview

In the *src* directory are:
- *registration.cpp*: pairwise registration pipeline (in the main function);
- *utils.cpp*: implementation of individual modules (e.g. RANSAC, ICP) of the pipeline;

In the *include* directory are function declarations and definitions.

-------------------------------
### Usage
To run the pipeline: <br>
1. Compile:
```
mkdir build
cmake..
cd build && make
```
2. Execute: 
to execute, several parameters need to be passed in as arguments:
```
./registration $[NORMAL SEARCH PARAMETER] $[FPFH SEARCH PARAMTER] $[RANSAC THRESHOLD] $[ICP TRANSFORMATION EPSILON] $[ICP CORRESPONDENCE DISTANCE] $[ICP EUCLIDEAN EPSILON] $[ICP OUTLIER THRESHOLD] $[POSE MATRIX FILE] $[PATH TO DATASET]
```
It is convenient to invoke the executable using Python, below is an example:
```
import os

program='registration'

normal_param = str(FLOAT) // Radius Search
fpfh_param = str(FLOAT)
// if you use K-search, the cpp source file needs to be modified and re-compiled

ransac_threshold = str(FLOAT)
icp_trans=str(FLOAT)
icp_corresD=str(FLOAT)
icp_eucli=str(FLOAT)
icp_outlierT=str(FLOAT)

pose_path=$PATH_TO_RESULT_FILE
dataset_path=$PATH_TO_DATASET

cmd = './' + program + ' ' + normal_param + ' ' + fpfh_param + ' ' + ransac_threshold + ' ' \
+ icp_trans + ' ' + icp_corresD + ' ' + icp_eucli + ' ' + icp_outlierT + ' ' + \
pose_path + ' ' + dataset_path

os.system(cmd)
```

-------------------------------
### To-do:
- [ ] Add more modules for different stages in the pipeline;
- [ ] Figure out a better way to pass the arguments;

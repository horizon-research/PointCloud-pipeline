# PointCloud-pipeline
Pipelines for a range of Point Cloud processing tasks (currently our focus is on the fundamental registration task).

---------------
## Registration
We adopt the standard pair-wise registration pipeline [registration pipeline](http://pointclouds.org/documentation/tutorials/registration_api.php) to perform pairwise point cloud registration:
```
Input: One pair of Point Cloud Frames
Output: Transformation Matrix
Pipeline: 
1. Preprocessing: filetering and downsampling -> 
2. Normal Estimation ->
3. Keypoint Selection ->
4. Feature Descriptor Calculation -> 
5. Correspondence Estimation among keypoints -> \
6. Correspondence Rejection (Ransac) -> 
7. Iterative Closest Point 
```
In the pipeline, the first 6 stages together could be viewed as a **coarse estimation** process, which generates a initial estimation result (usually rough) at the end of the *Correspondence Rejection* stage. Then, the last stage, which could be considered as a **fine estimation** process, calculates the final result in an iterative fashion given the initial estimation.

The following section briefly lists various options for each stage in our pipeline.
#### 1. Preprocessing
#### 2. Normal Estimation
Paramter: raidus (radius search for neighboring search) / (kNN search for neighboring search).
#### 3. Keypoint Selection
Configurations / Modules: **NARF**, **SIFT**, **HARRIS**.

#### 4. Feature Descriptor Calculation
Configurations / Modules: **SHOT**, **FPFH**, **VFH**, **PFH** and **3DSC**.
Paramter: raidus (radius search for neighboring search) / (kNN search for neighboring search).

#### 5. Correspondence Estimation
Paramter: whether to adopt reciprocal estimation or not.

#### 6. Correspondence Rejection
Paramter: rejection threshold.

#### 7. Iterative Closest Point
Configurations / Modules: **SVD**, **Non-Linear** for the solver.
Paramter: convergence criteria.

---------------
For the registration task: 
- We have been testing our implementation with the [Kitti Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php);
- It has been demonstrated that the estimation results could vary largely depending on the parameter setting of different modules in the pipeline, e.g., the radius or K for nearest neighbor search in the Normal Estimation stage.
- The implementation is based on [PCL](http://www.pointclouds.org/) and [FLANN](https://github.com/mariusmuja/flann).

---------------

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
5. Correspondence Estimation among keypoints -> 
6. Correspondence Rejection (Ransac) -> 
7. Iterative Closest Point 
```
In the pipeline, the first 6 stages together could be viewed as a **coarse estimation** process, which generates an initial estimation result (usually rough) at the end of the *Correspondence Rejection* stage. 
Then, the last stage, which could be considered as a **fine estimation** process, calculates the final result in an iterative fashion given the initial estimation.

Our implementation is designed to be flexible, in a way that each individual stage is re-configurable. For example, in the *feature descriptor calculation* stage, one could choose different descriptors such as FPFH and SHOT by given a specification parameter.

The following section briefly lists various kernel and paramter options for each stage in our pipeline.

| Stages   |      Kernels      |  Parameters |
|:--------:|:-----------------:|:-----------:|
| Preprocessing | x | x |
| Normal Estimation | x |  radius / K |
| Keypoint Selection | **NARF**, **SIFT**, **HARRIS** | x |
| Feature Descriptor Calculation | **SHOT**, **FPFH**, **3DSC** | radius / K |
| Correspondence Estimation | x | whether to estimate reciprocally or not |
| Correspondence Rejection | x | rejection threshold |
| Iterative Closest Point | **SVD**, **Non-Linear** | convergence criteria |

---------------
For the registration task: 
- We have been testing our implementation with the [Kitti Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php);
- It has been demonstrated that the estimation results could vary largely depending on the parameter setting of different modules in the pipeline, e.g., the radius or K for nearest neighbor search in the Normal Estimation stage.
- The implementation is based on [PCL](http://www.pointclouds.org/) and [FLANN](https://github.com/mariusmuja/flann).

---------------
## Preparation
To-do:
- [ ] Instruction on installing (build and compile from source) [PCL](http://www.pointclouds.org/) and [FLANN](https://github.com/mariusmuja/flann) in the user space on a Linux system.

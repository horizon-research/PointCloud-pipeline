# PointCloud-pipeline
Point Cloud Processing pipelines.

---------------
### Registration
- Currently we are following the most common [registration pipeline](http://pointclouds.org/documentation/tutorials/registration_api.php) to perform pairwise point cloud registration:
```
Input: Point Cloud Data
Output: transformation matrix
Pipeline: 1. Preprocessing -> 2. Feature Extraction -> 3. Correspondence Estimation -> \
4. Coarse Alignment (Ransac) -> 5. Fine Alignment (ICP) 
```

#### 1. Preprocessing
In this stage we perform **Downsampling** and **Filtering** on the original Point Cloud.
#### 2. Feature Extraction
In this stage, we first **detect keypoints** and calculate **feature descriptors**. <br>
Though there are many options out there, currently we are using **NARF** for Keypoint Detection and **FPFH** as Feature Descriptor.
#### 3. Correspondence Estimation
To find corresponding point pairs from the **keypoint** sets (not the original point cloud sets).
#### 4. Coarse Alignment
The goal is to get an initial result of the transformation matrix. Currently we are using **Ransac** on Keypoints and their correspondences.
#### 5. Fine Alignment
The goal is to find the final transformation matrix given the initial result from the previous stage. 

---------------

- We are testing our implementation with the [Kitti Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php);
- Our experiments have demonstrated that the results could vary largely depends on the parameter setting of individual modules in the pipeline;
- Library: [PCL 1.7](http://pointclouds.org/).
---------------

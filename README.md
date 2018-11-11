# PointCloud-pipeline
Point Cloud Processing pipelines.

---------------
### Registration
- Currently we are following the most common [registration pipeline](http://pointclouds.org/documentation/tutorials/registration_api.php) to perform pairwise point cloud registration:
```
Input: Point Cloud Data
Output: transformation matrix
Pipeline: Downsampling + Filtering -> Keypoint Detection -> Feature Extraction -> 
Correspondence Estimation -> Coarse Alignment (Ransac) -> Fine Alignment (ICP) 
```

- We are testing our implementation with the [Kitti Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php);
- Our experiments have demonstrated that the results could vary largely depends on the parameter setting of individual modules in the pipeline;
- Library: [PCL](http://pointclouds.org/).

---------------

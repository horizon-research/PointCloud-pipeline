### Point Cloud Registration
-------------------------------
#### Overview
Use [PCL](http://pointclouds.org/) to perform pairwise registration.

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
To execute, several parameters need to be passed in as arguments:
```
./registration $[NORMAL SEARCH PARAMETER] $[FPFH SEARCH PARAMTER] $[RANSAC THRESHOLD] $[ICP TRANSFORMATION EPSILON] $[ICP CORRESPONDENCE DISTANCE] $[ICP EUCLIDEAN EPSILON] $[ICP OUTLIER THRESHOLD] $[POSE MATRIX FILE] $[PATH TO DATASET]
```

-------------------------------

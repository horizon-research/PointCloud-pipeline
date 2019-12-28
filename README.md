Point Cloud Registration Pipeline 
=================

Artifact for paper "Tigris: Architecture and Algorithms for 3D Perception in Point Clouds" (MICRO'19): a general and flexible Point Cloud Registration pipeline built with Point Cloud Library ([PCL](http://pointclouds.org/)). <br><br>Our implementation is excellent for development and evaluation of point-cloud registration algorithms as it is representative as well as highly configurable: components of the pipeline could be configured with various PCL kernels, or even your own implementations! 

Pipeline Overview
------------------
[Registration](https://en.wikipedia.org/wiki/Point_set_registration) is a key building block in virtually all point cloud-based applications. It is a process that finds the transformation matrix that aligns two point cloud frames to form a globally consistent point cloud. 

![image](https://user-images.githubusercontent.com/19209239/62985785-c70b5280-be06-11e9-89c8-0cf9a698dd5a.png)
As shown in the figure above, our pipeline adopts a common two-phase design consisting of an initial estimation phase and a fine-tuning phase. The first phase performs an initial estimation of the transformation matrix, which is then fine-tuned in the second phase until the convergence.

Get Started (Before Running)
------------------
0. **Prequisites**:<br>
Make sure to [install PCL](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php) and its [dependencies](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php#dependencies) correctly (the visualization module is not needed here). [PCL 1.9.0](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.9.0) is preferred (we will use the shared objects only). 

1. **Project Settings**:<br> 
Three **IMPORTANT** specifications need to be made in the **CMakeLists.txt** in order to build the project successfully.<br>
	
	**A). set the path to **PCLConfig.cmake** correctly** <br>
	In the third line of the default CMakeLists.txt, set the path to **PCLConfig.cmake** correctly: <br>
	```set(PCL_DIR $PATH_TO_CONFIG_FILE)``` <br>
	
	Below is an example on my machine: the path to the **PCLConfig.cmake** file is ```/u/txu17/local/pcl-1.9.0/share/pcl-1.9``` (benefits of installing PCL under user space: more control of where and what to build, no need for sudo access).<br>
	<img src="https://user-images.githubusercontent.com/19209239/62990401-da73e900-be19-11e9-86e0-79c43455a790.png" width=80% height=80%>

	So in the CMakeLists.txt file, PCL_DIR is set to that path:
	<img src="https://user-images.githubusercontent.com/19209239/62990407-e19af700-be19-11e9-9174-c142db9e35db.png" width=80% height=80%> <br>
	(CMake has default paths to find PCLConfig.cmake, but they are not necessarily where PCLConfig.cmake is on your machine)). <br>
	
	**B). set the **PCL version** correctly** <br>
	Set the PCL version in the ```find_package``` line.
	
	**C). set the included headers correctly** <br>
	In the ```include_directories("./pcl-1.7;$PATH_TO_EIGEN;" include)``` line, please make sure ```./pcl-1.7``` is always there, and change ```$PATH_TO_EIGEN``` to the path to EIGEN. Example when EIGEN is installed at ```/usr/include/eigen3```: 
	<img src="https://user-images.githubusercontent.com/19209239/62991625-3ccee880-be1e-11e9-81ee-0636337b067e.png" width=80% height=80%>
	
2. **Dataset**: <br>
Two functions, i.e., ```loadTXTFile``` and ```loadBINFile```, are provided to load point cloud data (the implementations are in ```src/utils.cpp```): <br>
    ```loadTXTFile``` is designed to load points in text files. An input file is considered to be a point cloud frame, in which each line should contain the coordinates of a point, formatted as: ```x_coord,y_corrd,z_corrd```. Example below: <br>
    <img src="https://user-images.githubusercontent.com/19209239/63061628-c5539480-bec3-11e9-830c-0eb2d0e50b02.png" width=80% height=80%>
    
    ```loadBINFile``` is designed specifically for KITTI binary data. It also does the coordinate system transformation. <br>

    Depending on the format of the input data, you could modify ```src/pc_pipeline.cpp``` to switch between these two functions.

3. **Pipeline configuration**:<br>
There are several ways to configure the registration pipeline, i.e., choosing kernels and specifying algorithm parameters of different stages: <br>

	**a). To use the configuration file** <br>
	By default, the configuration file is named **'config.txt'** (in the **bin** directory). It specifies kernel parameters as well as some paths in the format of: ```$KEY: $VALUE```. For example, this line in the configuration file:<br> ```Normal_Search_Radius: 0.75``` <br> sets the parameter **Search Radius** in the **Normal Estimation** stage to 0.75. <br><br>
	Below is a complete list of parameters that could be set by the configuration file (which could be extended!): <br>
	
	|  Parameter Name |  Stage   |  Meaning  |     Values   |
	|:---------------:|:--------:|:---------:|:------------:|
	| Normal_Search_Radius | Normal Estimation | Search radius | A floating point, e.g., 0.75 | 
	| Normal_Use_Customized_KDTree | Normal Estimation | Use customized KD Tree or not | {true, false} |
	| Normal_Max_Leaf_Size | Normal Estimation | Max leaf size for customized KD-Tree | An integer number, e.g., 32|	
	| Key_Point_Detection_Module | Key-point Detection | Detection algorithm | {NARF, SIFT, HARRIS} | 
	| Feature_Search_Radius | Descriptor Calculation | Search radius | A floating point, e.g., 0.85 | 
	| Feature_Module | Descriptor Calculation | Feature descriptor type | {SHOT, FPFH} | 
	| Corr_Est_Use_Reciprocal_Search | Key-Point Correspondence Estimation | To do reciprocal search or not | {true, false} | 
	| Ransac_Threshold | Correspondence Rejection | Distance threshold | A floating point, e.g., 0.20 | 
	| Ransac_Max_Iteration | Correspondence Rejection | Maximum iteration number | An integer, e.g., 10000| 
	| ICP_Solver | Fine-Tuning (ICP) | Solver type | {SVD, LM} | 
	| ICP_Max_Iteration | Fine-Tuning (ICP) | Maximum iteration number | An integer, e.g., 15 | 
	| ICP_Use_Ransac | Fine-Tuning (ICP) | To do rejection in each iteration or not | {true, false} | 
	| ICP_Use_Reciprocal_Search | Fine-Tuning (ICP) | To do reciprocal search in each iteration or not | {true, false} |
	| ICP_Transformation_Epsilon | Fine-Tuning (ICP) | Transformation matrix epsilon  | A floating point, e.g., 1e-9 |
	| ICP_Max_Correspondence_Distance | Fine-Tuning (ICP) | Correspondences with higher distances will be ignored | A floating point, e.g., 1.20|
	| ICP_Euclidean_Fitness_Epsilon | Fine-Tuning (ICP) | Euclidean distance difference epsilon | A floating point, e.g., 1e-6 |
	| ICP_Ransac_Outlier_Rejection_Threshold | Fine-Tuning (ICP) |  | An integer number, e.g., 120 |
	| ICP_Use_Customized_KDTree | Fine-Tuning (ICP) | Use customized KD-Tree or not | {true, false} |
	| ICP_Max_Leaf_Size | Fine-Tuning (ICP) | Max leaf size for customized KD-Tree | An integer number, e.g., 32 |
	| Approx_Radius_Search_Para | Normal Estimation | Threshold for approximated radius search | A floating point under 1.0, e.g., 0.20 |
	| Approx_Nearest_Search_Para | Fine-Tuning (ICP) | Threshold for approximated nearest search | A floating point, e.g., 1.20 |	
	| Save_Approx_Data | Normal Estimation / Fine-Tuning (ICP) | Record data for approximation search or not| {true, false} |	


	**b). To modify the source code**
	- [ ] change to different PCL kernels: ICP example.
	- [ ] add implementations.

Build & Run
------------
1. Switch to the project directory (where the Makefile is) if you are not, and ```make```. By default, if everything set correctly, it will build the project and generate the executable in the **bin** directory.
2. Enter the **bin** directory, and set the **config.txt**. Especially, please set the path to dataset correctly as the example below (suppose the data is at ```/data/pointCloud/```):
	<img src="https://user-images.githubusercontent.com/19209239/63047716-fbcce780-bea2-11e9-9702-fa8c6ff199fc.png" width=80% height=80%>
	
3. Make sure the configurations are all set, then execute ```./pc_pipeline``` (default name). Hopefully the output will be similar to what's shown below: <br>
	<img src="https://user-images.githubusercontent.com/19209239/62992636-32165280-be22-11e9-89fc-37230272b133.png" width=80% height=80%>

++ Switch to the **kdtree_module** directory to take a look at our implementation (mainly based on FLANN's) of KD-Tree construction and search. If the parameter ```Normal_Use_Customized_KDTree``` is set to be True, the pipeline would use our version of KD-Tree search in the Normal Estimation Stage. ```ICP_Use_Customized_KDTree``` works samely.

Publication
------------------
This project implements the configurable point cloud registration pipeline described in the following paper:

T. Xu*, B. Tian*, and Y. Zhu, "Tigris: Architecture and Algorithms for 3D Perception in Point Clouds", In Proc. of MICRO, 2019. (* co-primary authors)

Please kindly consider citing this paper in your publications if it helps your research.
```
@inproceedings{xu2019tigris,
  title={Tigris: Architecture and Algorithms for 3D Perception in Point Clouds},
  author={Xu, Tiancheng and Tian, Boyuan and Zhu, Yuhao},
  booktitle={Proceedings of the 52th International Symposium on Microarchitecture},
  year={2019},
  organization={ACM}
}
```
  
Acknowledgement
------------------

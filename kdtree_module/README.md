### KDTree_Module

---------------
#### Overview
This is our own implementation of KD-Tree, i.e., the Customized KD-Tree, used in the pipeline. <br>The implementation is based on FLANN's but with a few changes. It is identical to the content in<br> 
``` ./pcl-1.7/pcl/horizon_kdtree.h``` 
<br>

---------------
#### To use
- Compile & Link:
``` make kdtree```
- Run:
``` ./kdtree```
- Tree Construction and Search settings can be changed in ```./kdtree.cpp```
---------------

#### Relevant
- [FLANN](https://github.com/mariusmuja/flann/blob/master/src/cpp/flann/algorithms/kdtree_single_index.h)

# ORB-SLAM2-with-saved-point-cloud

This repository makes `.ply` file (Point Cloud Map) with [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [PointCloudLibrary](https://github.com/PointCloudLibrary/pcl).  
When you execute ORB-SLAM2 with this repository, you could get point cloud map result.  

## Requirement
The test was conducted **Ubuntu 18.04** with the specifications shown in the table below.  

|Library or Pakage|Version|  
|:----------: |:----------:|
|OpenCV|3.4.0|
|Eigen|3.1.1|
|PCL||

These reposiory needs same requirement with [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2#2-prerequisites) 

## How to Build  

There are many subfolders to build.  
In `/Thirdparty` folder, there are `/g2o` and `/DBoW2`.  

You should build these folders first.  
When you clone this repository,  
`git clone https://github.com/Taeyoung96/ORB-SLAM2-with-saved-point-cloud.git`  

You move to path `~ ORBSLAM2_with_save_pointcloud/Thirdparty`.  
Build 2 subfolders.  

```
cd g2o  
mkdir build
cd build
cmake ..
make
make install
```

and  

```
cd DBoW2
mkdir build
cd build
cmake ..
make
make install
```






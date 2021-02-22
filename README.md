# ORB-SLAM2-with-saved-point-cloud

This repository makes `.ply` file (Point Cloud Map) with [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [PointCloudLibrary](https://github.com/PointCloudLibrary/pcl).  
When you execute ORB-SLAM2 with this repository, you could get point cloud map result.  

## Requirement
The test was conducted **Ubuntu 18.04** and **CUDA 10.2** with the specifications shown in the table below.  

|Library or Pakage|Version|  
|:----------: |:----------:|
|OpenCV|3.4.0|
|Eigen|3.1.1|
|PCL||
|CMake|3.10.2|
|Boost|1.65|


These reposiory needs same requirement with [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2#2-prerequisites).  

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

Go back to path `~ ORBSLAM2_with_save_pointcloud/`  

You should build libuvc.  

```
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
```

`cd ..` and go back to path `~ ORBSLAM2_with_save_pointcloud/`  
Uncompress vocabulary file.  
```
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
```

#### Then build whole ORB-SLAM2 system.  
`cd ..` and go back to path `~ ORBSLAM2_with_save_pointcloud/`  
```
chmod +x build.sh
sh build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **rgbd_my**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.  


## Prepare a Dataset













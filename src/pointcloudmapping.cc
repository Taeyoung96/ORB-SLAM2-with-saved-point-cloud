/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include "PointCloude.h"

namespace ORB_SLAM2
{
	// class PointCloude;	// forward declaration

	int currentloopcount = 0;
	PointCloudMapping::PointCloudMapping(double resolution_, double meank_, double thresh_) : loopbusy(false), bStop(false)
	{
		this->resolution = resolution_;
		this->meank = meank_;
		this->thresh = thresh_;
		statistical_filter.setMeanK(meank);
		statistical_filter.setStddevMulThresh(thresh);
		voxel.setLeafSize(resolution, resolution, resolution);
		globalMap = boost::make_shared< PointCloud >();

		viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
	}

	void PointCloudMapping::shutdown()
	{
		{
			unique_lock<mutex> lck(shutDownMutex);
			shutDownFlag = true;
			keyFrameUpdated.notify_one();
		}
		viewerThread->join();
	}

	void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth, int idk, vector<KeyFrame*> vpKFs)
	{
		cout << "receive a keyframe, id = " << kf->mnId << endl;
		unique_lock<mutex> lck(keyframeMutex);
		keyframes.push_back(kf);
		cout << "keyframes size : " << keyframes.size() << endl;
		currentvpKFs = vpKFs;
		cout <<"complete currentvpKFs" << endl;
		PointCloude pointcloude;
		
		pointcloude.pcID = idk;
		pointcloude.T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
		pointcloude.pcE = generatePointCloud(kf, color, depth);
		
		cout <<"complete generatePointCloud" << endl;
		pointcloud.push_back(pointcloude);
		cout << "pointcloud size : " <<pointcloud.size() << endl;
		cout << "success push back" <<  endl;
		keyFrameUpdated.notify_one();
	}

	pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
	{
		PointCloud::Ptr tmp(new PointCloud());
		// point cloud is null ptr
		//cout << "point cloud is null ptr" << endl;
		for (int m = 0; m < depth.rows; m += 3)
		{
			for (int n = 0; n < depth.cols; n += 3)
			{
				float d = depth.ptr<float>(m)[n];
				if (d < 0.01 || d>5)	// 원래 d > 5 였음
					continue;
				PointT p;
				p.z = d;
				p.x = (n - kf->cx) * p.z / kf->fx;
				p.y = (m - kf->cy) * p.z / kf->fy;

				p.b = color.ptr<uchar>(m)[n * 3];
				p.g = color.ptr<uchar>(m)[n * 3 + 1];
				p.r = color.ptr<uchar>(m)[n * 3 + 2];

				tmp->points.push_back(p);
			}
		}

		/*Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
		PointCloud::Ptr cloud(new PointCloud);
		pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
		cloud->is_dense = false;*/

		cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<tmp->points.size()<<endl;

		return tmp;
	}

	void PointCloudMapping::viewer()
	{
		/*pcl::visualization::CloudViewer viewer("viewer");
		while (1)
		{

			{
				unique_lock<mutex> lck_shutdown(shutDownMutex);
				if (shutDownFlag)
				{
					break;
				}
			}
			{
				unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
				keyFrameUpdated.wait(lck_keyframeUpdated);
			}

			// keyframe is updated 
			size_t N = 0;
			{
				unique_lock<mutex> lck(keyframeMutex);
				N = keyframes.size();
			}

			if (loopbusy || bStop)
			{

				continue;
			}

			if (lastKeyframeSize == N)
				cloudbusy = false;

			cloudbusy = true;
			for (size_t i = lastKeyframeSize; i < N; i++)
			{


				PointCloud::Ptr p(new PointCloud);
				pcl::transformPointCloud(*(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());

				*globalMap += *p;



			}

			// depth filter and statistical removal 
			PointCloud::Ptr tmp1(new PointCloud);

			statistical_filter.setInputCloud(globalMap);
			statistical_filter.filter(*tmp1);

			voxel.setInputCloud(tmp1);
			voxel.filter(*globalMap);
			viewer.showCloud(globalMap);
			//cout << "show global map, size=" << N << "   " << globalMap->points.size() << endl;
			lastKeyframeSize = N;
			cloudbusy = false;

		}*/
	}

void PointCloudMapping::save()
{
	bStop = true;
	cout << "Saving Final Map" << endl;
	PointCloud::Ptr tmp1(new PointCloud);
	for (int i = 0;i < currentvpKFs.size();i++)
	{
		for (int j = 0;j < pointcloud.size();j++)
		{
			if (pointcloud[j].pcID == currentvpKFs[i]->mnId)
			{
				Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose());
				PointCloud::Ptr cloud(new PointCloud);
				pcl::transformPointCloud(*pointcloud[j].pcE, *cloud, T.inverse().matrix());
				*tmp1 += *cloud;

				continue;
			}
		}
	}
	cout << "Finish Final Map" << endl;
	PointCloud::Ptr tmp2(new PointCloud());

	statistical_filter.setInputCloud(tmp1);
	statistical_filter.filter(*tmp2);


	voxel.setInputCloud(tmp2);
	voxel.filter(*tmp1);
	globalMap->swap(*tmp2);

	pcl::io::savePLYFile("Final_Map.ply", *globalMap);
	cout << "Final Map Created" << endl;
	//loopbusy = false;
	//loopcount++;
}

void PointCloudMapping::updatecloud()
{
	if (!cloudbusy)
	{
		loopbusy = true;
		cout << "start loop map point" << endl;
		PointCloud::Ptr tmp1(new PointCloud);
		for (int i = 0;i < currentvpKFs.size();i++)
		{
			for (int j = 0;j < pointcloud.size();j++)
			{
				if (pointcloud[j].pcID == currentvpKFs[i]->mnFrameId)
				{
					Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose());
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud(*pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 += *cloud;

					continue;
				}
			}
		}
		cout << "finish loop map" << endl;
		PointCloud::Ptr tmp2(new PointCloud());
		voxel.setInputCloud(tmp1);
		voxel.filter(*tmp2);
		globalMap->swap(*tmp2);

		//pcl::io::savePLYFile("result111.ply", *globalMap);
		//cout << "globalMap save finished" << endl;
		loopbusy = false;
		loopcount++;
	}
}

}


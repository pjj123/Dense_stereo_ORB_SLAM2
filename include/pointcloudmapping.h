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

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping( double resolution_ );

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat color_l, cv::Mat color_r );
    void shutdown();
    void viewer();
    bool isFinished();
    void setFinished();
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat colorl, cv::Mat colorr);
    PointCloud::Ptr generatePointCloud_pjj(KeyFrame* kf,cv::Mat colorl,cv::Mat colorr);
    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;
    bool mbFinished;
    mutex mMutexFinish;
    bool    shutDownFlag    =false;
    mutex   shutDownMutex;

    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;

    // data to generate point clouds
    //vector<KeyFrame*>       keyframes;
    //vector<cv::Mat>         colorImgsl;
    //vector<cv::Mat>         colorImgsr;
    deque<KeyFrame*> keyframes;
    deque<cv::Mat>   colorImgsl;
    deque<cv::Mat> colorImgsr;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.01;
    pcl::VoxelGrid<PointT>  voxel;
};

#endif // POINTCLOUDMAPPING_H

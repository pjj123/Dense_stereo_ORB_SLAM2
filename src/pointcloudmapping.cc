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
#include <pcl/io/pcd_io.h>
#include <boost/make_shared.hpp>
#include "ncc.h"
//#define octomap_debug

#ifdef octomap_debug
#include<octomap/octomap.h>
#include<octomap/ColorOcTree.h>
#include<octomap/math/Pose6D.h>
#endif

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    mbFinished=false;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
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

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat color_l, cv::Mat color_r)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
  // cout<<"color_l.rows="<<color_l.rows<<color_r.rows<<"color_l.cols="<<color_l.cols<<color_r.cols<<"color_l.type()="<<color_l.type()<<endl;
    CV_Assert(color_l.rows==color_r.rows&&color_l.cols==color_r.cols&&color_l.type()==color_r.type());
    //color_l.convertTo(color_l,CV_32F,1/255.0f);
    //color_r.convertTo(color_r,CV_32F,1/255.0f);
   // cv::Canny(color_l,color_l,5,20);
    //cv::Canny(color_r,color_r,5,20);
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgsl.push_back( color_l.clone() );
    colorImgsr.push_back( color_r.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat colorl, cv::Mat colorr)
{
    PointCloud::Ptr tmp( new PointCloud() );
    cv::Mat disl=cv::Mat::zeros(colorl.rows,colorl.cols,CV_8UC1);
    cv::Mat disr=cv::Mat::zeros(colorr.rows,colorr.cols,CV_8UC1);
    cv::Mat depth=cv::Mat::zeros(colorl.rows,colorl.cols,CV_32F);
    NCC* nccmtd1=new NCC(colorl,colorr,90,1,5);
     nccmtd1->buildcv();
     //cout<<"nccmtd1.hei="<<nccmtd1->hei<<endl;
     disl=nccmtd1->getdis();
     //cout<<"nccmtd1,wei="<<nccmtd1->wid<<endl;
     NCC* nccmtd=new NCC(colorl,colorr,90,0,5);
     nccmtd->buildcv();
     //cout<<"nccmtd.hei="<<nccmtd->hei<<endl;
     disr=nccmtd->getdis();
    //cout<<"nccmtd.wid="<<nccmtd->wid<<endl;
     for(int y=0;y<depth.rows;y++){
        uchar* pl=(uchar*)disl.ptr<uchar>(y);
        float* pdepth=(float*)depth.ptr<float>(y);
        uchar* pr=(uchar*)disr.ptr<uchar>(y);
     for(int x=0;x<depth.cols;x++){
         if(pl[x]<1)
             continue;
            int a=pl[x];
            if(x<a){
                pl[x]=(uchar)0;
            }else  if(abs(pl[x]-pr[x-a])>1){
                pl[x]=(uchar)0;
           }  else{
                pdepth[x]=((float)pl[x]+(float)pr[x-a])/2;
               // cout<<"pl[x]="<<(float)pl[x]<<"pr[x-a]="<<(float)pr[x-a]<<"pdepth[x]="<<pdepth[x]<<endl;
           }
     }
    }

    for ( int m=0; m<depth.rows; m++ )
    {
        for ( int n=0; n<depth.cols; n++ )
        {
            float d =depth.ptr<float>(m)[n];
            if (d < 1)
                continue;
            PointT p;
            p.z = kf->fx*kf->mb/d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
   if(colorl.channels()==3){
            p.b = colorl.ptr<uchar>(m)[n*3];
            p.g = colorl.ptr<uchar>(m)[n*3+1];
            p.r = colorl.ptr<uchar>(m)[n*3+2];
}else{
       p.b = colorl.ptr<uchar>(m)[n];
       p.g =0;
       p.r = 0;
   }
            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloud_pjj(KeyFrame* kf,cv::Mat colorl,cv::Mat colorr){
    cv::imshow("1",colorl);
    cv::imshow("2",colorr);
    cv::waitKey(0);
}

void PointCloudMapping::viewer()
{
#ifdef octomap_debug
    octomap::ColorOcTree tree(0.05);
#endif
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
#ifdef cotomap_debug
                   tree.write("pjj_octomap.ot");
#endif
                   setFinished();
                      if(isFinished())
                          break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i=i+2 )
        {
            double time=static_cast<double>(getTickCount());
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgsl[i], colorImgsr[i] );
          //  keyframes.pop_front();
           // colorImgsl.pop_front();
           // colorImgsr.pop_front();
            double time1=static_cast<double>(getTickCount());
            double detatime=(time1-time)/cv::getTickFrequency();
            cout<<"dedaodianyun time="<<detatime<<".s"<<endl;
  #ifdef octomap_debug
            cv::Mat t=keyframes[i]->GetTranslation();
            octomap::Pointcloud cloud_octo;
            for(auto q:p->points)
                cloud_octo.push_back(q.x,q.y,q.z);
            tree.insertPointCloud(cloud_octo,octomap::point3d(t.at<float>(0,0),t.at<float>(1,0),t.at<float>(2,0)));
            for(auto q:p->points)
                tree.integrateNodeColor(q.x,q.y,q.z,q.r,q.g,q.b);
            double time2=static_cast<double>(getTickCount());
             detatime=(time2-time1)/cv::getTickFrequency();
            cout<<"tree time="<<detatime<<".s"<<endl;
          #endif
            *globalMap += *p;
        }
        for(size_t i=lastKeyframeSize;i<N;i++)
        {
            keyframes.pop_front();
            colorImgsl.pop_front();
            colorImgsr.pop_front();
        }
       // keyframes.shrink_to_fit();
        //colorImgsl.shrink_to_fit();
        //colorImgsr.shrink_to_fit();
#ifdef octomap_debug
        double time3=static_cast<double>(getTickCount());

       tree.updateInnerOccupancy();
       //tree.write("pjj_octomap.ot");
       double time4=static_cast<double>(getTickCount());
      double  detatime=(time4-time3)/cv::getTickFrequency();
      cout<<"tree.update time="<<detatime<<".s"<<endl;
  #endif
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        cout<<globalMap->points.size()<<endl;
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        pcl::io::savePCDFileASCII("test_pcd.pcd",*globalMap);
        //pcl::io::savePCDFile("depth_result.pcd",globalMap);

       double time5=static_cast<double>(getTickCount());
        viewer.showCloud( globalMap );
         double time6=static_cast<double>(getTickCount());
         double  detatime=(time6-time5)/cv::getTickFrequency();
           cout<<"viewer time="<<detatime<<".s"<<endl;
        cout << "show global map, size=" << globalMap->points.size()<< endl;
        lastKeyframeSize = 0;
    }
}

bool PointCloudMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}
void PointCloudMapping::setFinished(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished=true;
}

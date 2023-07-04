/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
// #include"../../../include/System.h"
#include"System.h"

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h> 
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}


    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

void PosePublishThread(ORB_SLAM2::System &SLAM)
{
        ros::NodeHandle nh;
        cv::Mat Tcw ;
        cv::Mat R ;
        cv::Mat t ;
        cv::Mat rvec ;
        tf2::Quaternion q;
        

        ros::Publisher PosePublisher = nh.advertise<geometry_msgs::PoseStamped>("Pose", 1);
        ros::Publisher PathPublisher = nh.advertise<nav_msgs::Path>("Path", 1);
        geometry_msgs::PoseStamped pose_stamped;
        nav_msgs::Path path_msg; 
        path_msg.header.frame_id = "map"; 
        while (ros::ok() && !SLAM.GetPointCloud()->isFinished())
        {
            Tcw = SLAM.GetPointCloud()->WaitAndReturnT();
            if (Tcw.empty()) 
            {
                continue; // or break, return, etc.
            }
            R = Tcw(cv::Rect(0, 0, 3, 3));
            t = Tcw(cv::Rect(3, 0, 1, 3));
            // Convert rotation matrix to quaternion
            tf2::Matrix3x3 m(R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
            R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
            R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));

            m.getRotation(q);
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";

            pose_stamped.pose.position.x = t.at<float>(0);
            pose_stamped.pose.position.y = t.at<float>(1);
            pose_stamped.pose.position.z = t.at<float>(2);

            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
            PosePublisher.publish(pose_stamped);
            PathPublisher.publish(path_msg);

            SLAM.GetPointCloud()->ResetInterrupt();

            ros::spinOnce();

        }
    
}

void PointCloudPublishThread(ORB_SLAM2::System &SLAM)
{

        ros::NodeHandle nh;

        ros::Publisher PointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("PointCloud", 1);
        sensor_msgs::PointCloud2 output;

        // ros::Rate rate(10);
        while (ros::ok() && !SLAM.GetPointCloud()->isFinished())
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
            SLAM.GetPointCloud()->CloneGlobalPC(tmp);
            // std::unique_lock<mutex> lockGlobalPC(std::ref(SLAM.GetPointCloud()->GetGlobalPCMutex()));
            // {
            //     pcl::toROSMsg(*SLAM.GetPointCloud()->GetGlobalPC(),output);
                
            // }
            
            pcl::toROSMsg(*tmp,output);
            output.header.stamp = ros::Time::now();
            output.header.frame_id = "map";
            PointCloudPublisher.publish(output);
            ros::spinOnce();
            // rate.sleep();
            std::chrono::milliseconds duration(2000);  // 0.1秒
            std::this_thread::sleep_for(duration);

        }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    std::thread PointCloudPublishing(PointCloudPublishThread,std::ref(SLAM));
    std::thread PosePublishing(PosePublishThread,std::ref(SLAM));
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    PointCloudPublishing.join();
    PosePublishing.join();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout<<"正在保存点云"<<endl;
    SLAM.SavePointCloud();
    cout<<"保存成功"<<endl;

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}



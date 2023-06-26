#include <functional>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <mutex>
#include <pcl/point_types.h>
#include "PointCloudMapping.h"
#include "KeyFrame.h"
#include "Frame.h"
#include <thread>
#include <pcl/visualization/pcl_visualizer.h>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Converter.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <X11/Xlib.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pangolin/pangolin.h>
#include <condition_variable> 

namespace ORB_SLAM2
{
    using namespace std;
    PointCloudMapping::PointCloudMapping()
    {
        // mpGlobalCloud = pcl::PointCloud<pcl::PointXYZRGBA>::makeShared();
        // pcl::make_shared<pcl::PointXYZRGBA>( );

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
        mpGlobalCloud = cloud1;
        mbFinishRequested = false;
        mflag = 0;
        mkk = 0;
        mflagLC = 0;
        mflagThread = 0;
        mbUpdateCloudFinished = false;
        USEPCLVIEWER = false;
        XInitThreads();
        UnsetInsertStop();
    }

    void PointCloudMapping::DisplayPointCloud()
    {
        mbFinished = false;
        KeyFrame *pKF;

        int lastN = 0;
        bool flag = 0;
        int updateN = 0;
        bool flag3 = 0;
        int N = 0;
        int iii = 0;
        if (USEPCLVIEWER)
        {
            pcl::visualization::CloudViewer *viewer = new pcl::visualization::CloudViewer("PCviewer");
            viewer->runOnVisualizationThread(std::bind(&PointCloudMapping::VisualizationCallback, this, std::placeholders::_1), "VisualizationCallback");
        }
        // std::thread RosThread = new thread(&PointCloudMapping::PointCloudPublish, this);
        while (1)
        {

            unique_lock<std::mutex> lock(mMutexPCKF, defer_lock);
            lock.lock();
            N = mvpKF.size();
            lock.unlock();

            unique_lock<mutex> lock222(mMutexPCForLC, defer_lock);
            lock222.lock();
            bool flaglc = mflagLC;
            lock222.unlock();
            // 闭环更新地图点

            if (flaglc)
            {
                // 停止插入关键帧(仅停止插入到点云类，tracking仍然进行)
                SetInsertStop();
                mbUpdateCloudFinished = false;
                UpdatePointCloud();
                flaglc = 0;
                lock222.lock();
                mflagLC = 0;
                lock222.unlock();
                mbUpdateCloudFinished = true;
                // 恢复插入关键帧
                UnsetInsertStop();
                cout << "結束update" << endl;
            }
            // 为每一个插入的关键帧生成点云，并且在pangolin中显示出当前帧的点云
            if (N > lastN)
            {
                for (; lastN < N; lastN++)
                {
                    cout<<"N:"<<N<<endl;
                    cout<< "lastN" <<lastN<<endl;

                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    lock.lock();
                    pKF = mvpKF[lastN];
                    lock.unlock();
                    cv::Mat Twc = pKF->GetPoseInverse();
                    cloud = GenerateCloud(pKF);
                    if(cloud == nullptr)
                        continue;
                    std::unique_lock<std::mutex> lockGlobal(mMutexGlobalPC);
                    {
                        *mpGlobalCloud += *cloud;
                    }
                    std::unique_lock<std::mutex> lockT(mMutexRosT);
                    {
                        mRosTwc = Twc;
                        mInterrupt = true;
                        mCV.notify_one();
                    }
                }
            }
            if (CheckFinish() && mbUpdateCloudFinished) // 检查是否request finish并且闭环更新是否完成，完成闭环并且有结束请求就结束
            {
                cout << "点云线程结束" << endl;
                break;
            }
        }
        SetFinish(); // 标志线程已经finished
        // RosThread.join();
    }

    void PointCloudMapping::UpdatePointCloud()
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cout << "检测到闭环，正在更新点云" << endl;
        cout << "一共有 ： " << mvpKF.size() << endl;
        for (int i = 0; i < mvpKF.size(); i++)
        {
            KeyFrame *pKF;
            unique_lock<std::mutex> Updatelock(mMutexPCKF);
            {
                pKF = mvpKF[i];
            }

            if (pKF->isBad())
            {
                cout << "坏帧" << endl;
                continue;
            }
            else
            {
                cout << "闭环更新帧 ：" << i << endl;
                cv::Mat Twc = pKF->GetPoseInverse();
                Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp3(new pcl::PointCloud<pcl::PointXYZRGBA>);
                tmp2 = pKF->GetPointCloud();
                pcl::transformPointCloud(*tmp2, *tmp3, T1.matrix());
                std::unique_lock<std::mutex> lockGlobalUpdate(mMutexGlobalPC);
                {
                    *mpGlobalCloud += *tmp3;
                }
            }
        }
        // std::unique_lock<std::mutex> lockGlobalUpdate(mMutexGlobalPC);
        // mpGlobalCloud = tmp1;
        mflag = 0;
        // viewer->updatePointCloud(mpGlobalCloud);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GenerateCloud(KeyFrame *pKF)
    {
        if(pKF->isBad())
        {
            cout << "bad bad bad" << endl;
            return nullptr;
        }
        float z, x, y;
        cv::Mat Twc = pKF->GetPoseInverse();
        Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);
        cv::Mat Depth = pKF->mimDepthForPC;
        cv::Mat RGB = pKF->mimRGBForPC;
        pcl::PointXYZRGBA p;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

        cout << "正在生成帧ID ：" << pKF->mnId << "的点云" << endl;
        // if(pKF->mnId<10)
        // {
        //     cout << "empty" <<endl;
        //     return nullptr;
        // }
        for (int v = 0; v < Depth.rows; v++)
        {
            // cout << "hihihihihiihihsdsdsdsdsdsdi" <<endl;
            for (int u = 0; u < Depth.cols; u++)
            {
                // 获取每个像素的深度值
                // cout << "hihihihihiihihi" <<endl;
                z = Depth.at<float>(v, u);
                // cout << "hihihihihiihihfffffffffffffffffffff:wi" <<endl;
                if (z < 0.01 || z > 5)
                    continue;
                else if (std::isnan(z))
                    z = 0;
                x = (u - pKF->cx) * z * pKF->invfx;
                y = (v - pKF->cy) * z * pKF->invfy;

                p.b = RGB.ptr<uchar>(v)[u * 3];
                p.g = RGB.ptr<uchar>(v)[u * 3 + 1];
                p.r = RGB.ptr<uchar>(v)[u * 3 + 2];
                p.x = x;
                p.y = y;
                p.z = z;
                glPointSize(2);
                glBegin(GL_POINTS);
                // Extract RGB values
                uint8_t r = (p.rgba >> 16) & 0x0000ff;
                uint8_t g = (p.rgba >> 8) & 0x0000ff;
                uint8_t b = (p.rgba) & 0x0000ff;

                glColor3f(r / 255.0f, g / 255.0f, b / 255.0f); // Set the color for the point
                glVertex3d(p.x, p.y, p.z);                     // Draw the point
                glEnd();
                cloud->push_back(p);
                // cout << "finish" <<endl;
            }
        }

        // cout << "滤波中" << endl;
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        // vg.filter(*tmp);
        // cout << "滤波ed" << endl;
        vg.filter(*cloud);
        pKF->SetPointCloud(cloud); // 帧坐标下的点云
        pcl::transformPointCloud(*cloud, *cloud2, T1.matrix());
        pKF->mimDepthForPC.release();
        pKF->mimRGBForPC.release();
        // pKF->mimDepthOriginal.release();

        return cloud2;
    }

    void PointCloudMapping::InsertKeyFrame(KeyFrame *pKF)
    {
        // if(pKF->isBad())
        // {
        //     cout << "bad bad" << endl;
        //     return;
        // }
        if (!CheckInsertStop())
        {
            // cv::Mat Depth = pKF->mimDepthForPC;
            // mimDepthForPC = pKF->mimDepthForPC;
            // mvimDepthForPC.push_back(Depth);
            mkk++;
            // cout << "关键帧  : " << mkk << endl;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            cv::Mat DepthOri = pKF->mimDepthForPC;
            mvimDepthOriForPC.push_back(DepthOri);

            unique_lock<std::mutex> lock(mMutexPCKF);
            {
                mvpKF.push_back(pKF);
            }
        }
        else
        {
            cout << "正在update点云，不能插入KeyFrame" << endl;
        }
    }

    void PointCloudMapping::SavePointCloud()
    {

        if (!mflag)
        {
            pcl::io::savePCDFileBinary("afterLC.pcd", *mpGlobalCloud);
            cout << "点云数量 ： " << mpGlobalCloud->size() << endl;
            mflagThread = 1;
        }
    }

    bool PointCloudMapping::CheckFinish()
    {
        unique_lock<mutex> lockfinish(mMutexPCFinish);
        return mbFinishRequested;
    }

    void PointCloudMapping::RequestFinish()
    {
        unique_lock<mutex> lockfinish(mMutexPCFinish);
        mbFinishRequested = true;
    }

    bool PointCloudMapping::isFinished()
    {
        unique_lock<mutex> lockfinish(mMutexPCFinish);
        return mbFinished;
    }

    void PointCloudMapping::SetFinish()
    {
        unique_lock<mutex> lockfinish(mMutexPCFinish);
        mbFinished = true;
    }

    void PointCloudMapping::SetInsertStop()
    {
        unique_lock<mutex> lockinsert(mMutexPCInsertStop);
        mbInsertStop = true;
    }

    bool PointCloudMapping::CheckInsertStop()
    {
        unique_lock<mutex> lockinsert(mMutexPCInsertStop);
        return mbInsertStop;
    }

    void PointCloudMapping::UnsetInsertStop()
    {
        unique_lock<mutex> lockfinish(mMutexPCInsertStop);
        mbInsertStop = false;
    }
    void PointCloudMapping::VisualizationCallback(pcl::visualization::PCLVisualizer &viz)
    {
        // std::unique_lock<mutex> lockGlobal1(mMutexGlobalPC);

        // if (CheckInsertStop())
        //{
        // std::this_thread::sleep_for(0.5s);
        // cout << "不更新viewer" << std::endl;
        //}
        // else
        {
            // cout << "更新viewer" << std::endl;
            std::unique_lock<mutex> lockGlobal1(mMutexGlobalPC);
            {
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(mpGlobalCloud);
                if (!viz.updatePointCloud(mpGlobalCloud, rgb, "GlobalPointCloud"))
                {
                    viz.addPointCloud(mpGlobalCloud, rgb, "GlobalPointCloud");
                }
            }
        }
    }
    // void PointCloudMapping::PointCloudPublish()
    // {
    //     int zero = 0;
    //     char **nullpointer = nullptr;
    //     ros::init(zero, nullpointer, "PointCloudPublishing");
    //     ros::NodeHandle nh;

    //     ros::Publisher PointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("PointCloud", 1);
    //     sensor_msgs::PointCloud2 output;

    //     ros::Rate rate(10);
    //     while (ros::ok() && !isFinished())
    //     {
    //         std::unique_lock<mutex> lockRosGlobalPC(mMutexGlobalPC);
    //         {
    //             pcl::toROSMsg(*mpGlobalCloud, output);
    //         }
    //         PointCloudPublisher.publish(output);
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    // }
    void PointCloudMapping::SetGlobalPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &PC)
    {
        std::unique_lock<std::mutex> lockGlobalPCinSet(mMutexGlobalPC);
        {
            mpGlobalCloud = PC;
        }
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GetGlobalPC()
    {
        // std::unique_lock<mutex> lockGlobalPCinGet(mMutexGlobalPC);
        {
            return mpGlobalCloud;
        }
    }
    void PointCloudMapping::AddGlobalPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &PC)
    {
        std::unique_lock<mutex> lockGlobalPCinAdd(mMutexGlobalPC);
        {
            *mpGlobalCloud += *PC;
        }
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::CloneGlobalPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &PC)
    {
        std::unique_lock<mutex> lockGlobalPCinClone(mMutexGlobalPC);
        {
            // *PC = *mpGlobalCloud;
            pcl::copyPointCloud(*mpGlobalCloud,*PC);
        }
        return PC;
    }

    std::mutex& PointCloudMapping::GetGlobalPCMutex()
    {
        return mMutexGlobalPC;
    }
    std::mutex& PointCloudMapping::GetRosTMutex()
    {
        return mMutexRosT;
    }
    void PointCloudMapping::ResetInterrupt()
    {
        std::unique_lock<std::mutex> lock(mMutexRosT);
        mInterrupt = false;
    }
    cv::Mat PointCloudMapping::WaitAndReturnT()
    {
        std::unique_lock<std::mutex> lock(mMutexRosT);
        mCV.wait(lock,[this]{return mInterrupt;});
        return mRosTwc;
    }

}

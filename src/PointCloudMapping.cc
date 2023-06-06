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

namespace ORB_SLAM2
{
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

        UnsetInsertStop();
    }

    void PointCloudMapping::DisplayPointCloud()
    {
        mbFinished = false;
        // pangolin初始化相关
        // pangolin::CreateWindowAndBind("Map Viewer", 1024, 768);
        // glEnable(GL_DEPTH_TEST);
        // glEnable(GL_BLEND);
        // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // pangolin::OpenGlRenderState vis_camera(
        //     pangolin::ProjectionMatrix(1024, 768, 535.4, 539.2, 512, 384, 0.1, 1000),
        //     pangolin::ModelViewLookAt(0, 0, -1.5, 0, 0, 0, 0.0, -1.0, 0.0));

        // // Add named OpenGL viewport to window and provide 3D Handler
        // pangolin::View &vis_display = pangolin::CreateDisplay()
        //                                   .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        //                                   .SetHandler(new pangolin::Handler3D(vis_camera));

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCViewer"));
         pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
         pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(mpGlobalCloud);
         viewer->addPointCloud<pcl::PointXYZRGBA>(mpGlobalCloud, rgb, "cloud");
         viewer->setBackgroundColor(0, 0, 0);
         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
         viewer->initCameraParameters();

        KeyFrame *pKF;

        // pcl::visualization::CloudViewer viewer("Cloud Viewer");
        int lastN = 0;
        bool flag = 0;
        int updateN = 0;
        bool flag3 = 0;
        int N = 0;
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        int iii = 0;
        while (1)
        {

            viewer->spinOnce(500);
            // pangolin清除上一帧信息

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
                UpdatePointCloud(viewer);
                flaglc = 0;
                lock222.lock();
                mflagLC = 0;
                lock222.unlock();
                // usleep(5000);
                mbUpdateCloudFinished = true;
                // 恢复插入关键帧
                UnsetInsertStop();
            }
            // 为每一个插入的关键帧生成点云，并且在pangolin中显示出当前帧的点云
            if (N > lastN)
            {
                for (; lastN < N; lastN++)
                {

                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    lock.lock();
                    pKF = mvpKF[lastN];
                    lock.unlock();
                    cv::Mat Twc = pKF->GetPoseInverse();
                    // Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);
                    // pangolin::OpenGlMatrix m(T1.matrix());
                    //vis_camera.Follow(m);
                    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                    // vis_display.Activate(vis_camera);
                    cloud = GenerateCloud(pKF);

                    *mpGlobalCloud += *cloud;
                    // 在pangolin中画出这个点云

                    // for (auto &p : mpGlobalCloud->points)
                    // {
                    //     uint8_t r = (p.rgba >> 16) & 0x0000ff;
                    //     uint8_t g = (p.rgba >> 8) & 0x0000ff;
                    //     uint8_t b = (p.rgba) & 0x0000ff;

                    //     glColor3f(r / 255.0f, g / 255.0f, b / 255.0f);
                    //     glVertex3d(p.x, p.y, p.z);
                    // }
                    //glBegin(GL_POINTS);
                    // for (iii=0; iii < cloud->points.size(); iii++)
                    // {
                    //     pcl::PointXYZRGBA &p = mpGlobalCloud->points[iii];
                    //     uint8_t r = (p.rgba >> 16) & 0x0000ff;
                    //     uint8_t g = (p.rgba >> 8) & 0x0000ff;
                    //     uint8_t b = (p.rgba) & 0x0000ff;

                    //     glColor3f(r / 255.0f, g / 255.0f, b / 255.0f);
                    //     glVertex3d(p.x, p.y, p.z);
                    // }
                    // //glEnd();

                    // pangolin::FinishFrame();
                    // cloud = nullptr;
                    viewer->updatePointCloud(mpGlobalCloud);
                    // cloud = nullptr;
                    
                    // viewer.showCloud(mpGlobalCloud);
                }
                // lastN = N ;
            }
            // viewer.showCloud(mpGlobalCloud);
            if (CheckFinish() && mbUpdateCloudFinished) // 检查是否request finish并且闭环更新是否完成，完成闭环并且有结束请求就结束
            {
                cout << "点云线程结束" << endl;
                break;
            }
            // viewer.showCloud(mpGlobalCloud);
            //  std::this_thread::sleep_for(1000ms);
        }
        SetFinish(); // 标志线程已经finished
    }

    void PointCloudMapping::UpdatePointCloud(const pcl::visualization::PCLVisualizer::Ptr& viewer)
    {
        viewer->close();
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
                *tmp1 += *tmp3;
            }
        }
        mpGlobalCloud = tmp1;
        mflag = 0;
        // viewer->updatePointCloud(mpGlobalCloud); 
        viewer->spinOnce(500);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GenerateCloud(KeyFrame *pKF)
    {
        float z, x, y;
        cv::Mat Twc = pKF->GetPoseInverse();
        Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);
        cv::Mat Depth = pKF->mimDepthForPC;
        cv::Mat RGB = pKF->mimRGBForPC;
        pcl::PointXYZRGBA p;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // cout << "正在生成帧ID ：" << pKF->mnId << "的点云" << endl
        //<< endl;
        for (int v = 0; v < Depth.rows; v++)
        {
            for (int u = 0; u < Depth.cols; u++)
            {
                // 获取每个像素的深度值
                z = Depth.at<float>(v, u);
                if (z < 0.01 || z > 5)
                    continue;
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
            }
        }

        // cout << "滤波中" << endl;
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.012f, 0.012f, 0.012f);
        vg.filter(*tmp);
        // cout << "滤波ed" << endl;

        pKF->SetPointCloud(tmp); // 帧坐标下的点云
        pcl::transformPointCloud(*tmp, *cloud2, T1.matrix());
        pKF->mimDepthForPC.release();
        pKF->mimRGBForPC.release();
        pKF->mimDepthOriginal.release();

        return cloud2;
    }

    void PointCloudMapping::InsertKeyFrame(KeyFrame *pKF)
    {
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
            // cloud = GenerateCloud(pKF);

            // unique_lock<std::mutex> lock(mMutexPC);
            // *mpGlobalCloud += *cloud;

            // float z, x, y;
            // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
            // pcl::PointXYZRGBA p;
            // cv::Mat Twc = pKF->GetPoseInverse();
            // Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);

            // for (int v = 0; v < DepthOri.rows; v++)
            // {
            //     for (int u = 0; u < DepthOri.cols; u++)
            //     {
            //         // 获取每个像素的深度值
            //         z = DepthOri.at<float>(v, u);
            //         if (z < 0.01 || z > 5)
            //             continue;
            //         x = (u - pKF->cx) * z * pKF->invfx;
            //         y = (v - pKF->cy) * z * pKF->invfy;

            //         p.b = pKF->mimRGBForPC.ptr<uchar>(v)[u * 3];
            //         p.g = pKF->mimRGBForPC.ptr<uchar>(v)[u * 3 + 1];
            //         p.r = pKF->mimRGBForPC.ptr<uchar>(v)[u * 3 + 2];
            //         p.x = x;
            //         p.y = y;
            //         p.z = z;
            //         cloud->push_back(p);
            //     }
            // }
            // pKF->SetPointCloud(cloud);

            // pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
            // mkk++;
            // if (!pKF->mimRGBForPC.empty())
            // {
            //     // pcl::transformPointCloud( *cloud, *cloud2, T1.inverse().matrix());
            //     pcl::transformPointCloud(*cloud, *cloud2, T1.matrix());
            //     cout << "关键帧  : " << mkk << endl;
            //     *mpGlobalCloud += *cloud2;
            //     // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
            //     //  sor.setInputCloud(tmp);
            //     //  sor.setLeafSize(0.1f, 0.1f, 0.1f);
            //     //  //unique_lock<std::mutex> lock(mMutexPC);
            //     //  sor.filter(*mpGlobalCloud);
            //     // pcl::io::savePCDFileBinary("test_pcd25.pcd", *cloud2);
            // }
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
}
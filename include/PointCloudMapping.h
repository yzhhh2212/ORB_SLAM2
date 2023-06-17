#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <iostream>
#include<vector>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include<vtkAutoInit.h>
#include <pangolin/display/viewport.h>
#include <pangolin/display/attach.h>
#ifdef Success 
#undef Success
#endif

VTK_MODULE_INIT(vtkRenderingOpenGL2);

VTK_MODULE_INIT(vtkInteractionStyle);

VTK_MODULE_INIT(vtkRenderingFreeType);

//避免和Eigen冲突




class KeyFrame;

namespace ORB_SLAM2
{
class PointCloudMapping
{
    
public:
    //构造函数
    PointCloudMapping();
    //显示点云
    void DisplayPointCloud();

    //插入关键帧
    void InsertKeyFrame(KeyFrame *pKF);

    //更新点云
    void UpdatePointCloud();

    //保存点云
    void SavePointCloud();
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GenerateCloud(KeyFrame *pKF);

    void PointCloudPublish();

public:
    cv::Mat mimDepthForPC; 
    std::vector<cv::Mat> mvimDepthForPC;
    std::vector<cv::Mat> mvimDepthOriForPC;
    std::vector<KeyFrame*> mvpKF;
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mpGlobalCloud;
    void VisualizationCallback(pcl::visualization::PCLVisualizer& viz);
    //对Globalpointcloud的操作函数
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetGlobalPC();
    void SetGlobalPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &PC);
    void AddGlobalPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloneGlobalPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &PC);

    bool mflag ;

    std::mutex mMutexPC;
    std::mutex mMutexPCKF;
    std::mutex mMutexPCForLC;
    
    bool mflagLC;
    bool mflagThread;
    int mkk;
    bool mbUpdateCloudFinished;
    void RequestFinish();
    bool isFinished();
    bool CheckInsertStop();
    bool USEPCLVIEWER;
    bool USERVIZ;
    bool ROSMODE;
protected:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mpGlobalCloud;
    bool mbFinishRequested;
    bool mbFinished;
    bool mbInsertStop;
    bool CheckFinish();
    void SetFinish();
    void SetInsertStop();
    void UnsetInsertStop();
    std::mutex mMutexPCFinish;
    std::mutex mMutexPCStop;
    std::mutex mMutexPCInsertStop;
    std::mutex mMutexGlobalPC;


};






}


#endif
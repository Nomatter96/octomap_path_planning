#pragma once

#include <mutex>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sl_zed/Camera.hpp>

namespace server {

class PCLServer
{
public:
    PCLServer(const int aPort = 11111,
              const float aLeafSizeX = 0.01f,
              const float aLeafSizeY = 0.01f,
              const float aLeafSizeZ = 0.01f)
        : mPort(aPort)
        , mViewer("PCL")
    {
        mVoxelGridFilter.setLeafSize(aLeafSizeX, aLeafSizeY, aLeafSizeZ);			
    };
    ~PCLServer() {};	
    void Run();
		
private:
    void StartZED();
    void Start();
    void CloseZED();
    inline float ConvertColor(float aColorIn);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        GetRegSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *aPointCLoud);

    sl::Camera mZed;
    sl::Mat mDataCloud;
    bool mStopSignal;
    bool mHasData;
    int mPort;
    std::thread mZedCallback;
    std::mutex mMutexInput;
    pcl::visualization::CloudViewer mViewer;		
    long long int mSizeBuff;
    pcl::VoxelGrid<pcl::PointXYZRGB> mVoxelGridFilter;
};

}
#pragma once

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <sl_zed/Camera.hpp>

namespace server {

class PCLServer
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
    typedef pcl::PointXYZRGB PCLPoint;

    PCLServer(const int aPort,
              const float aLeafSizeX,
              const float aLeafSizeY,
              const float aLeafSizeZ);
    ~PCLServer() {};
    void Run();
		
private:
    inline float ConvertColor(float aColorIn);
    inline static void UpdateMinKey(const octomap::OcTreeKey& aIn,
                                          octomap::OcTreeKey& aMin);
    inline static void UpdateMaxKey(const octomap::OcTreeKey& aIn,
                                          octomap::OcTreeKey& aMax);
    std::shared_ptr<pcl::visualization::PCLVisualizer> 
        CreateRGBVisualizer(PCLPointCloud::ConstPtr aCloud);
    octomap::point3d GetOriginVector(sl::Transform& aPoseData);
    void CloseZED();
    void FilterGroundPlane(const PCLPointCloud& aCloud,
                           PCLPointCloud& aGround,
                           PCLPointCloud& aNonGround);
    void InsertCloud(const PCLPointCloud::Ptr aCloud);
    void InsertScan(const octomap::point3d aOriginVector,
                    const PCLPointCloud& aGround,
                    const PCLPointCloud& aNonGround);
    void TransformAsMatrix(sl::Transform& aPoseData, Eigen::Matrix4f& aMPose);
    void SetOcTree(PCLPointCloud::Ptr aCloud);
    void Start();
    void StartZED();
    void UpdateBBX();
    static void sUpdateKey(const octomap::OcTreeKey& aIn,
                                    octomap::OcTreeKey& aMin,
                                    octomap::OcTreeKey& aMax);

    bool mHasData;
    bool mStopSignal;
    double mMaxRange;
    int mPort;
    octomap::ColorOcTree mOcTree;
    octomap::KeyRay mKeyRay;
    octomap::OcTreeKey mBBXMin;
    octomap::OcTreeKey mBBXMax;
    pcl::VoxelGrid<PCLPoint> mVoxelGridFilter;
    sl::Camera mZed;
    sl::Mat mDataCloud;
    std::mutex mMutexInput;
    std::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    std::thread mZedCallback;
};

}

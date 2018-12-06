#pragma once

#include <mutex>
#include <string>
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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transform.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>

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
    inline static void UpdateMinKey(const OcTreeKey& aIn, OcTreeKey& aMin);
    inline static void UpdateMaxKey(const OcTreeKey& aIn, OcTreeKey& aMax);
    std::shared_ptr<pcl::visualization::PCLVisualizer> 
        CreateRGBVisualizer(PCLPointCloud::ConstPtr aCloud);    
    void CloseZED();
    void FilterGroundPlane(const PCLPointCloud& aCloud,
                           PCLPointCloud& aGround,
                           PCLPointCloud& aNonGround);
    void InsertCloud(const PCLPointCLoud::Ptr Cloud);
    void InsertScan(const tf::Point& aSensorOriginTf,
                    const PCLPointCloud& aGround,
                    const PCLPointCloud& aNonGround);
    void SetOcTree(PCLPointCloud::Ptr aCloud);
    void Start();
    void StartZED();

    bool mCompressMap;
    bool mFilterGroundPlane;
    bool mFilterSpeckles;
    bool mHasData;
    bool mIncrementalUpdate;
    bool mLatchedTopics;
    bool mPublishFreeSpace;
    bool mStopSignal;
    bool mUseColoredMap;
    bool mUseHeightMap;
    double mColorFactor;
    double mGroundFilterAngle;
    double mGroundFilterDistance;
    double mGroundFilterPlaneDistance;
    double mMaxRange;
    double mMinSizeX;
    double mMinSizeY;
    double mOccupancyMaxZ;
    double mOccupancyMinZ;
    double mPointCloudMaxX;
    double mPointCloudMaxY;
    double mPointCloudMaxZ;
    double mPointCloudMinX;
    double mPointCloudMinY;
    double mPointCloudMinZ;
    double mRes;
    int mPort;
    nav_msgs::OccupancyGrid mGridMap;
    octomap::ColorOcTree mOcTree;
    octomap::KeyRay mKeyRay;
    octomap::OcTreeKey mUpdateBBXMin;
    octomap::OcTreeKey mUpdateBBXMax;
    pcl::VoxelGrid<PCLPoint> mVoxelGridFilter;
    ros::NodeHandle mNh;
    ros::Publisher mBinaryMapPub;
    ros::Publisher mFmarkerPub;
    ros::Publisher mFullMapPub;
    ros::Publisher mMapPub;
    ros::Publisher mMarkerPub;
    ros::Publisher mPointCloudPub;
    sl::Camera mZed;
    sl::Mat mDataCloud;
    std::mutex mMutexInput;
    std::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    std::string mWorldFrameId;
    std::string mBaseFrameId;
    std::thread mZedCallback;
    std_msgs::ColorRGBA mColor;
    std_msgs::ColorRGBA mColorFree;
    tf::TransformListener mTfListener;
    unsigned mTreeDepth;
    unsigned mMaxTreeDepth;
};

}

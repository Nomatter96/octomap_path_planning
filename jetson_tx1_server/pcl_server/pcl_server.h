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

#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

namespace server {

class PCLServer
{
public:
    PCLServer(const int aPort = 11111,
              const float aLeafSizeX = 0.01f,
              const float aLeafSizeY = 0.01f,
              const float aLeafSizeZ = 0.01f);
    ~PCLServer() {};	
    void Run();
		
private:
    inline float ConvertColor(float aColorIn);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        GetRegSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *aPointCLoud); 
    std::shared_ptr<pcl::visualization::PCLVisualizer> 
        createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr aCloud);    
    void CloseZED();
    void DisplayCurvature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr aCloud, pcl::PointCloud<pcl::Normal>::Ptr aNormals);
    void DisplayDistanceMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr aCloud, float* aDistanceMap);
    void DisplayEuclideanClusters(const pcl::PointCloud<pcl::PointXYZRGB>::CloudVectorType &aClusters);
    void DisplayPlanarRegions(std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > &aRegions);
    void ObjSegAlg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aCloud);    
    void Start();
    void StartZED();

    bool mDisplayCurvature;
    bool mDisplayDistanceMap;
    bool mDisplayNormals;
    bool mHasData;
    bool mStopSignal;
    bool mUseClustering;
    bool mUsePlanarRefinement;
    int mPort;
    //pcl::visualization::CloudViewer mViewer;
    pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>::Ptr mEdgeAwareComparator;
    pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr mEuclideanClusterComparator;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> mNe;
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mMps;
    pcl::VoxelGrid<pcl::PointXYZRGB> mVoxelGridFilter;
    sl::Camera mZed;
    sl::Mat mDataCloud;
    sl::Plane mPlane;
    std::mutex mMutexInput;
    std::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    std::thread mZedCallback;
};

}

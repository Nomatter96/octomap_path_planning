#include "pcl_server.h"

using boost::asio::ip::tcp;
using namespace sl;
using namespace std;

namespace server {

PCLServer::PCLServer(const int aPort,
                     const float aLeafSizeX,
                     const float aLeafSizeY,
                     const float aLeafSizeZ)
    : mPort(aPort)
{

    
    mVoxelGridFilter.setLeafSize(aLeafSizeX, aLeafSizeY, aLeafSizeZ);

    mDisplayCurvature = true;
    mDisplayDistanceMap = false;
    mDisplayNormals = false;
    mUseClustering = true;
    mUsePlanarRefinement = true;

    mNe.setNormalEstimationMethod (mNe.COVARIANCE_MATRIX);
    mNe.setMaxDepthChangeFactor (0.05f);
    mNe.setNormalSmoothingSize (10.0f);
    mEdgeAwareComparator.reset (new pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>());
    mEuclideanClusterComparator = pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr(new pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>());
    mMps.setMinInliers (5000);
    mMps.setAngularThreshold (pcl::deg2rad (3.0));
    mMps.setDistanceThreshold (0.05);
}

void
PCLServer::Run()
{
    InitParameters initParams;
    initParams.camera_resolution = RESOLUTION_VGA;
    initParams.camera_fps = 30;
    initParams.coordinate_units = UNIT_METER;
    initParams.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    initParams.depth_mode = DEPTH_MODE_PERFORMANCE;

    ERROR_CODE err = mZed.open(initParams);
    if (err != SUCCESS) {
        cout << toString(err) << "\n";
        mZed.close();
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(mZed.getResolution().area());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        point_cloud_buff(new pcl::PointCloud<pcl::PointXYZRGB>);

    mViewer = createRGBVisualizer(p_pcl_point_cloud);

    StartZED();

    /*boost::asio::io_service ioService;
    tcp::endpoint endPoint(tcp::v4(), static_cast<unsigned short> (mPort));
    tcp::acceptor acceptor(ioService, endPoint);
    tcp::socket socket (ioService);

    cout << "Listening on port " << mPort << "...\n";
    acceptor.accept(socket);
    cout << "Client connected.\n";*/
    while (!mViewer->wasStopped()) {
        if (mMutexInput.try_lock()) {
            float *p_data_cloud = mDataCloud.getPtr<float>();
            int index = 0;
            for (auto &it : p_pcl_point_cloud->points) {
                if (!isValidMeasure(p_data_cloud[index]))
                    it.x = it.y = it.z = it.rgb = 0;  
                else {
                    it.x = p_data_cloud[index];
                    it.y = p_data_cloud[index + 1];
                    it.z = p_data_cloud[index + 2];
                    it.rgb = ConvertColor(p_data_cloud[index + 3]);
                }
                index += 4;
            }
            p_pcl_point_cloud->width = mDataCloud.getWidth();
            p_pcl_point_cloud->height = mDataCloud.getHeight();
            //mVoxelGridFilter.setInputCloud(p_pcl_point_cloud);
            //mVoxelGridFilter.filter(*point_cloud_buff);
            point_cloud_buff = p_pcl_point_cloud;
            /*unsigned int nrPoints = 
                static_cast<unsigned int>(point_cloud_buff->points.size());
            boost::asio::write(socket,
                boost::asio::buffer(&nrPoints, sizeof(nrPoints)));
            boost::asio::write(socket,
                boost::asio::buffer(&point_cloud_buff->points.front(),
                    nrPoints * 8 * sizeof(float)));*/
            mMutexInput.unlock();
            //mViewer->updatePointCloud(point_cloud_buff);
            ObjSegAlg(point_cloud_buff);
            mViewer->spinOnce(10);
            //mViewer.showCloud(point_cloud_buff);
        } else
            sleep_ms(1);
    }
    CloseZED();
}

void
PCLServer::StartZED()
{
    mStopSignal = false;
    mHasData = false;
    mZedCallback = thread(&PCLServer::Start, this);
    while (!mHasData)
        sleep_ms(1);
}

void
PCLServer::Start()
{
    while (!mStopSignal) {
        if (mZed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
            mMutexInput.lock();
            mZed.retrieveMeasure(mDataCloud, MEASURE_XYZRGBA);
            mMutexInput.unlock();
            mHasData = true;
        } else
            sleep_ms(1);
    }
}

void
PCLServer::CloseZED()
{
    mStopSignal = true;
    mZedCallback.join();
    mZed.close();
}

inline float
PCLServer::ConvertColor(float aColorIn)
{
    uint32_t colorUint = *(uint32_t *) & aColorIn;
    unsigned char *colorUchar = (unsigned char *) &colorUint;
    colorUint = ((uint32_t) colorUchar[0] << 16 |
        (uint32_t) colorUchar[1] << 8 | (uint32_t) colorUchar[2]);
    return *reinterpret_cast<float *> (&colorUint);
}

void
PCLServer::ObjSegAlg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aCloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr 
        normalCloud(new pcl::PointCloud<pcl::Normal>);
    mNe.setInputCloud(aCloud);
    mNe.compute(*normalCloud);
    float* distanceMap = mNe.getDistanceMap();
    boost::shared_ptr<pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>> eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>>(mEdgeAwareComparator);
    eapc->setDistanceMap (distanceMap);
    eapc->setDistanceThreshold (0.01f, false);
    
    vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB>>> regions;
    vector<pcl::ModelCoefficients> modelCoefficients;
    vector<pcl::PointIndices> inlierIndices;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    vector<pcl::PointIndices> labelIndices;
    vector<pcl::PointIndices> boundaryIndices;
    mMps.setInputNormals(normalCloud);
    mMps.setInputCloud (aCloud);
    if (mUsePlanarRefinement)
        mMps.segmentAndRefine(regions, modelCoefficients, inlierIndices, labels, labelIndices, boundaryIndices);
    else
        mMps.segment(regions);

    pcl::PointCloud<pcl::PointXYZRGB>::CloudVectorType clusters;
    if (mUseClustering && regions.size() > 0) {
        vector<bool> planeLabels;
        planeLabels.resize(labelIndices.size(), false);
        for (size_t i = 0; i <  labelIndices.size(); i++)
            if (labelIndices[i].indices.size() > 10000)
                planeLabels[i] = true;
        mEuclideanClusterComparator->setInputCloud(aCloud);
        mEuclideanClusterComparator->setLabels(labels);
        mEuclideanClusterComparator->setExcludeLabels(planeLabels);
        mEuclideanClusterComparator->setDistanceThreshold(0.01f, false);

        pcl::PointCloud<pcl::Label> euclideanLabels;
        vector<pcl::PointIndices> euclideanLabelIndices;
        pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGB, pcl::Label> euclideanSegmentation(mEuclideanClusterComparator);
        euclideanSegmentation.setInputCloud(aCloud);
        euclideanSegmentation.segment(euclideanLabels, euclideanLabelIndices);

        for (size_t i = 0; i < euclideanLabelIndices.size(); i++)
            if (euclideanLabelIndices[i].indices.size() > 1000) {
                pcl::PointCloud<pcl::PointXYZRGB> cluster;
                pcl::copyPointCloud(*aCloud, euclideanLabelIndices[i].indices, cluster);
                clusters.push_back(cluster);
            }
    }

    if (!mViewer->updatePointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*aCloud), "cloud")) {
        mViewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*aCloud), "cloud");
        mViewer->resetCameraViewpoint("cloud");
    }

    DisplayPlanarRegions(regions);

    if (mDisplayCurvature)
        DisplayCurvature(aCloud, normalCloud);
    else
        mViewer->removePointCloud ("curvature");

    if (mDisplayDistanceMap)
        DisplayDistanceMap(aCloud, distanceMap);
    else
        mViewer->removePointCloud("distance_map");

    if (mDisplayNormals) {
        mViewer->removePointCloud("normals");
        mViewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*aCloud), boost::make_shared<pcl::PointCloud<pcl::Normal>>(*normalCloud), 10, 0.05f, "normals");
        mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
    } else
        mViewer->removePointCloud("normals");

    DisplayEuclideanClusters(clusters);
}

void
PCLServer::DisplayPlanarRegions(std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > &aRegions)
{
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr contour(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < aRegions.size (); i++) {
        Eigen::Vector3f centroid = aRegions[i].getCentroid();
        Eigen::Vector4f model = aRegions[i].getCoefficients();
        pcl::PointXYZ pt1 = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0] + (0.5f * model[0]),
                                          centroid[1] + (0.5f * model[1]),
                                          centroid[2] + (0.5f * model[2]));
        sprintf(name, "normal_%d", unsigned(i));
        mViewer->addArrow(pt2, pt1, 1.0, 0, 0, false, name);
        
        contour->points = aRegions[i].getContour();
        sprintf (name, "plane_%02d", int(i));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(contour, red[i%6], grn[i%6], blu[i%6]);
        if(!mViewer->updatePointCloud(contour, color, name))
            mViewer->addPointCloud(contour, color, name);
        mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}

void
PCLServer::DisplayCurvature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr aCloud, pcl::PointCloud<pcl::Normal>::Ptr aNormals)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curvatureCloud = aCloud;
    for (size_t i  = 0; i < aCloud->points.size (); i++) {
        if (aNormals->points[i].curvature < 0.04) {
            curvatureCloud->points[i].r = 0;
            curvatureCloud->points[i].g = 255;
            curvatureCloud->points[i].b = 0;
        } else {
            curvatureCloud->points[i].r = 255;
            curvatureCloud->points[i].g = 0;
            curvatureCloud->points[i].b = 0;
        }
    }
    if (!mViewer->updatePointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*curvatureCloud), "curvature"))
        mViewer->addPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*curvatureCloud), "curvature");
}

void
PCLServer::DisplayDistanceMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr aCloud, float* aDistanceMap)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr distanceMapCloud = aCloud;
    for (size_t i  = 0; i < aCloud->points.size (); i++) {
        if (aDistanceMap[i] < 5.0) {
            distanceMapCloud->points[i].r = 255;
            distanceMapCloud->points[i].g = 0;
            distanceMapCloud->points[i].b = 0;
        } else {
            distanceMapCloud->points[i].r = 0;
            distanceMapCloud->points[i].g = 255;
            distanceMapCloud->points[i].b = 0;
        }
    }
    if (!mViewer->updatePointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*distanceMapCloud), "distance_map"))
        mViewer->addPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*distanceMapCloud), "distance_map");

}

void
PCLServer::DisplayEuclideanClusters(const pcl::PointCloud<pcl::PointXYZRGB>::CloudVectorType &aClusters)
{
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    for (size_t i = 0; i < aClusters.size (); i++) {
        sprintf(name, "cluster_%d", int(i));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(aClusters[i]), red[i%6], grn[i%6], blu[i%6]);
        if (!mViewer->updatePointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(aClusters[i]), color, name))
            mViewer->addPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(aClusters[i]), color, name);
        mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
        mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
}

}

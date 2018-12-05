#include "pcl_server.h"

using boost::asio::ip::tcp;
using namespace sl;
using namespace std;
using namespace octomap;

namespace server {

PCLServer::PCLServer(const int aPort,
                     const float aLeafSizeX,
                     const float aLeafSizeY,
                     const float aLeafSizeZ)
    : mPort(aPort)
    , mOcTree(0.1)
    , mPointCloudMaxX(numeric_limits<double>::max())
    , mPointCloudMaxY(numeric_limits<double>::max())
    , mPointCloudMaxZ(numeric_limits<double>::max())
    , mPointCloudMinX(-numeric_limits<double>::max())
    , mPointCloudMinY(-numeric_limits<double>::max())
    , mPointCloudMinZ(-numeric_limits<double>::max())
    , mFilterGroundPlane(false)
    , mWorldFrameId("/map")
    , mBaseFrameId("base_footprint")
    , mGroundFilterDistance(0.04)
    , mGroundFilterAngle(0.15)
    , mGroundFilterPlaneDistance(0.07)
    , mMaxRange(-1.0)
    , mCompressMap(true)
{
    mVoxelGridFilter.setLeafSize(aLeafSizeX, aLeafSizeY, aLeafSizeZ);
};

void
PCLServer::Run()
{
    InitParameters initParams;
    initParams.camera_resolution = RESOLUTION_VGA;
    initParams.camera_fps = 30;
    initParams.coordinate_units = UNIT_METER;
    initParams.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    initParams.depth_mode = DEPTH_MODE_QUALITY;

    ERROR_CODE err = mZed.open(initParams);
    if (err != SUCCESS) {
        cout << toString(err) << "\n";
        mZed.close();
        return;
    }

    PCLPointCloud::Ptr p_pcl_point_cloud(new PCLPointCloud);
    p_pcl_point_cloud->points.resize(mZed.getResolution().area());
    PCLPointCloud::Ptr 
        point_cloud_buff(new PCLPointCloud);

    mViewer = CreateRGBVisualizer(p_pcl_point_cloud);

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
            mVoxelGridFilter.setInputCloud(p_pcl_point_cloud);
            mVoxelGridFilter.filter(*point_cloud_buff);
            //point_cloud_buff = p_pcl_point_cloud;
            /*unsigned int nrPoints = 
                static_cast<unsigned int>(point_cloud_buff->points.size());
            boost::asio::write(socket,
                boost::asio::buffer(&nrPoints, sizeof(nrPoints)));
            boost::asio::write(socket,
                boost::asio::buffer(&point_cloud_buff->points.front(),
                    nrPoints * 8 * sizeof(float)));*/
            mMutexInput.unlock();
            mViewer->updatePointCloud(point_cloud_buff);
            SetOcTree(point_cloud_buff);
            mViewer->spinOnce(10);
        } else
            sleep_ms(1);
    }
    mOcTree.write("test.ot");
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

shared_ptr<pcl::visualization::PCLVisualizer>
PCLServer::CreateRGBVisualizer(PCLPointCloud::ConstPtr aCloud) 
{
    shared_ptr<pcl::visualization::PCLVisualizer> 
        viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<PCLPoint> rgb(aCloud);
    viewer->addPointCloud<PCLPoint>(aCloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

inline static void
PCLServer::UpdateMinKey(const OcTreeKey& aIn, OcTreeKey& aMin)
{
    for (int i = 0; i < 3; i++)
        aMin[i] = min(aIn[i], aMin[i]);
}

inline static void
PCLServer::UpdateMaxKey(const OcTreeKey& aIn, OcTreeKey& aMax)
{
    for (int i = 0; i < 3; i++)
        aMax[i] = max(aIn[i], aMax[i]);
}

void
PCLServer::SetOcTree(PCLPointCloud::Ptr aCloud)
{
    Pointcloud octoCloud;
    for (auto &it : aCloud->points)
        octoCloud.push_back(it.x, it.y, it.z);
    point3d sensorOrigin(0,0,0);
    mOcTree.insertPointCloud(octoCloud, sensorOrigin);
    for (auto &it : aCloud->points)
        mOcTree.integrateNodeColor(it.x, it.y, it.z, it.r, it.g, it.b);
    mOcTree.updateInnerOccupancy();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    mOcTree.getMetricMin(minX, minY, minZ);
    mOcTree.getMetricMax(maxX, maxY, maxZ);
    mUpdateBBXMin[0] = mOcTree.coordKey(minX);
    mUpdateBBXMin[1] = mOcTree.coordKey(minY);
    mUpdateBBXMin[2] = mOcTree.coordKey(minZ);
    mUpdateBBXMax[0] = mOcTree.coordKey(maxX);
    mUpdateBBXMax[1] = mOcTree.coordKey(maxY);
    mUpdateBBXMax[2] = mOcTree.coordKey(maxZ);
}

void
PCLServer::InsertCloud(const PCLPointCLoud::Ptr aCloud)
{
    tf::StampedTransform sensorToWorldTf;
    try {
        mTfListener.lookupTransform(mWorldFrameId, aCloud->header.frame_id,
            aCloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformExeption& aEx) {
        cerr << "Transform error of sensor data: " << ex.what();
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    pcl::PassThrough<PCLPoint> passX;
    passX.setFilterFieldName("x");
    passX.setFilterLimits(mPointCloudMinX, mPointCloudMaxX);
    pcl::PassThrough<PCLPoint> passY;
    passY.setFilterFieldName("y");
    passY.setFilterLimits(mPointCloudMinY, mPointCloudMaxY);
    pcl::PassThrough<PCLPoint> passZ;
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits(mPointCloudMinZ, mPointCloudMaxZ);

    PCLPointCloud pclGroud;
    PCLPointCloud pclNonGround;

    if (mFilterGroundPlane) {
        tf::StampedTransform sensorToBaseTf, baseToWorldTf;
        try {
            mTfListener.waitForTransform(mBaseFrameId, aCloud->header.frame.id,
                aCloud->header.stamp, ros::Duraction(0.2));
            mTfListener.lookupTransform(mBaseFrameId, aCloud->header.frame.id,
                aCloud->header.stamp, sensorToBaseTf);
            mTfListener.lookupTransform(mWorldFrameId, mBaseFrameId,
                aCloud->header.stamp, baseToWorldTf);
        } catch (tf::TransformException& aEx) {
            cerr << "Transform error for ground plane filter: " << aEx.what();
        }

        Eigen::Matrix4f sensorToBase, baseToWorld;
        pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
        pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

        pcl::transformPointCloud(aCloud, aCloud, sensorToBase);
        passX.setInputCloud(aCloud.makeShared());
        passX.filter(aCloud);
        passY.setInputCloud(aCloud.makeShared());
        passY.filter(aCloud);
        passZ.setInputCloud(aCloud.makeShared());
        passZ.filter(aCloud);
        FilterGroundPlane(aCloud, pclGround, pclNonGround);

        pcl::transformPointCloud(pclGround, pclGround, baseToWorld);
        pcl::transformPointCloud(pclNonGround, pclNonGround, baseToWorld);
    } else {
        pcl::transformPointCloud(aCloud, aCloud, sensorToWorld);
        passX.setInputCloud(aCloud.makeShared());
        passX.filter(aCloud);
        passY.setInputCloud(aCloud.makeShared());
        passY.filter(aCloud);
        passZ.setInputCloud(aCloud.makeShared());
        passZ.filter(aCloud);
        pclNonGround = aCloud;
        pclGround.header = aCloud.header;
        pclNonGround.header = aCloud.header;
    }

    InsertScan(sensorToWorldTf.getOrigin(), pclGround, pclNonGround);
    PublishAll(aCloud.header.stamp);
}

void
PCLServer::FilterGroundPlane(const PCLPointCloud& aCloud,
                             PCLPointCloud& aGround,
                             PCLPointCloud& aNonGround)
{
    aGround.header = aCloud.header;
    aNonGround.header = aCloud.header;
    if (aCloud.size() < 50) {
        aNonGround = aCloud;
        return;
    }
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(mGroundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(mGroundFilterAngle);

    PCLPointCloud cloudFiltered(aCloud);
    pcl::ExtractIndices<PCLPoint> extract;
    bool groundPlaneFound = false;
    while (cloudFiltered.size() > 10 && !groundPlaneFound) {
        seg.InputCloud(cloudFiltered.makeShared());
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) break;
        
        extract.setInputCloud(cloudFiltered.makeShared());
        extract.setIndices(inliers);
        if (abs(coefficients->value.at(3)) < mGroundFilterPlaneDistance) {
            extract.setNegative(false);
            extract.filter(aGround);
            if (inliers->indices.size() != cloudFiltered.size()) {
                extract.setNegative(true);
                PCLPointCloud cloudOut;
                extract.filter(cloudOut);
                aNonGround += cloudOut;
                cloudFiltered = cloudOut;
            }
            groundPlaneFound = true;
        } else {
            PCLPointCloud cloudOut;
            extract.setNegative(false);
            extract.filter(cloudOut);
            aNonGround += cloudOut;
            if (inliers->indices.size() != cloudFiltered.size()) {
                extract.setNegative(true);
                cloudOut.points.clear();
                extract.filter(cloudOut);
                cloudFiltered = cloudOut;
            } else
                cloudFiltered.points.clear();
        }
    }
    if (!groundPlaneFound) {
        pcl::PassThrough<PCLPoint> secondPass;
        secondPass.setFilterFieldName("z");
        secondPass.setFilterLimits(-mGroundFilterPlaneDistance,
                                    mGroundFilterPlaneDistance);
        secondPass.setInputCloud(aCloud.makeShared());
        secondPass.filter(aGround);
        secondPass.setFilterLimitsNegative(true);
        secondPass.filter(aNonGround);
    }
}

void
PCLServer::InsertScan(const tf::Point& aSensorOriginTf,
                      const PCLPointCloud& aGround,
                      const PCLPointCloud& aNonGround)
{
    point3d sensorOrigin = pointTfToOctomap(aSensorOriginTf);
    mOcTree.coordToKeyCheked(sensorOrigin, mUpdateBBXMin);
    mOcTree.coordToKeyCheked(sensorOrigin, mUpdateBBXMax);
    unsigned char* colors = new unsigned char[3];
    KeySet freeCells, occupiedCells;
    for (auto &it : aGround->points) {
        point3d point(it.x, it.y, it.z);
        if ((mMaxRange > 0.0) && ((point - sensorOrigin).norm() > mMaxRange))
            point = sensorOrigin + (point - sensorOrigin).normolized() *
                    mMaxRange;
        if (mOcTree.computeRayKeys(sensorOrigin, point, mKeyRay))
            freeCells.insert(mKeyRay.begin(), mKeyRay.end());
        OcTreeKey endKey;
        if (mOcTree.coordToKeyCheked(point, endKey)) {
            updateMinKey(endKey, mUpdateBBXMin);
            updateMaxKey(endKey, mUpdateBBXMax);
        } else {
            cerr << "Could not generate Key for endpoint";
    }
    for (auto &it : aNonGround->points) {
        point3d point(it.x, it.y, it.z);
        if((mMaxRange < 0.0) || ((point - sensorOrigin).norm() <= mMaxRange)) {
            if (mOcTree.computeRayKeys(sensorOrigin, point, mKeyRay))
                freeCells.insert(mKeyRay.begin(), mKeyRay.end());
            OcTreeKey key;
            if (m_octree->coordToKeyChecked(point, key)) {
                occupiedCells.insert(key);
            updateMinKey(key, mUpdateBBXMin);
            updateMaxKey(key, mUpdateBBXMax);
            }

            const int rgb = *reinterpret_cast<const int*>(&(it->rgb));
            colors[0] = ((rgb >> 16) & 0xff);
            colors[1] = ((rgb >> 8) & 0xff);
            colors[2] = (rgb & 0xff);
            mOcTree.averageNodeColor(it.x, it.y, it.z,
                                     colors[0], colors[1], colors[2]);
        } else {
            point3d newEnd = sensorOrigin + (point - sensorOrigin).normalized() * mMaxRange;
            if (mOcTree.computeRayKeys(sensorOrigin, newEnd, mKeyRay)){
                freeCells.insert(mKeyRay.begin(), mKeyRay.end());
                OcTreeKey endKey;
                if (mOcTree.coordToKeyChecked(newEnd, endKey)){
                    free_cells.insert(endKey);
                    updateMinKey(endKey, mUpdateBBXMin);
                    updateMaxKey(endKey, mUpdateBBXMax);
                } else
                    cerr << "Could not generate Key for endpoint ";
            }
        } 
    }
    for(KeySet::iterator it = freeCells.begin(), end=freeCells.end(); it != end; it++)
        if (occupiedCells.find(*it) == occupiedCells.end())
            mOcTree.updateNode(*it, false);
    for (KeySet::iterator it = occupiedCells.begin(), end=occupiedCells.end(); it!= end; it++)
        mOcTree.updateNode(*it, true);
    point3d minPt, maxPt;
    minPt = mOcTree.keyToCoord(mUpdateBBXMin);
    maxPt = mOcTree.keyToCoord(mUpdateBBXMax);
    if (mCompressMap)
        mOctree.prune();
    if (colors) {
        delete[] colors;
        colors = NULL;
    }
}

void
PCLServer::PublishAll() {
    size_t octomapSize = mOcTree->size();
    if (octomapSize <= 1) {
        cerr << "Nothing to publish, octree is empty";
        return;
    }
    bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);
}

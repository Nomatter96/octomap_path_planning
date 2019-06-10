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
    , mMaxRange(-1.0)
{
    //parameters zed cam
    InitParameters initParams;
    initParams.camera_resolution = RESOLUTION_VGA;
    initParams.camera_fps = 30;
    initParams.coordinate_units = UNIT_METER;
    initParams.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    initParams.depth_mode = DEPTH_MODE_MEDIUM;
    ERROR_CODE err = mZed.open(initParams);
    if (err != SUCCESS) {
        cout << toString(err) << "\n";
        mZed.close();
        return;
    }
    TrackingParameters trackingParameters;
    trackingParameters.initial_world_transform = sl::Transform::identity();
    trackingParameters.enable_spatial_memory = true;
    mZed.enableTracking(trackingParameters);

    //ocTree
    mOcTree.setProbHit(0.55);
    mOcTree.setProbMiss(0.4);
    mOcTree.setClampingThresMin(0.12);
    mOcTree.setClampingThresMax(0.97);

    mOPlanner = std::make_shared<Planner>();

    mVoxelGridFilter.setLeafSize(aLeafSizeX, aLeafSizeY, aLeafSizeZ);
};

void
PCLServer::Run()
{
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
            Pose cameraPose = mCameraPose;
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
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
            for (int i = 0; i <= 3; i++) {
                std::cout << cameraPose.pose_data(i, 0) << " " << cameraPose.pose_data(i, 1) << " " << cameraPose.pose_data(i, 2) << " " << cameraPose.pose_data(i, 3) << "\n";
            }
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
            InsertCloud(point_cloud_buff, cameraPose);
            mOPlanner->UpdateMap(mOcTree);
            //mOPlanner->Replan();
            mViewer->spinOnce(10);
        } else
            sleep_ms(1);
    }
    mOPlanner->SetStart(mCameraPose.pose_data(0, 3), mCameraPose.pose_data(1, 3), mCameraPose.pose_data(2, 3));
    mOPlanner->SetGoal(mCameraPose.pose_data(0, 3), mCameraPose.pose_data(1, 3), mCameraPose.pose_data(2, 3) - 0.5);
    mOPlanner->Replan();
    auto path = mOPlanner->GetSmoothPath();
    for (int i = 0; i < path.size(); i++) {
        std::cout << std::get<0>(path[i]) << " " << std::get<1>(path[i]) << " " << std::get<2>(path[i]) << "\n";
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
            mZed.getPosition(mCameraPose, REFERENCE_FRAME_WORLD);
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
    viewer->initCameraParameters();
    return (viewer);
}

void
PCLServer::UpdateBBX()
{
    double minX, minY, minZ, maxX, maxY, maxZ;
    mOcTree.getMetricMin(minX, minY, minZ);
    mOcTree.getMetricMax(maxX, maxY, maxZ);
    mBBXMin[0] = mOcTree.coordToKey(minX);
    mBBXMin[1] = mOcTree.coordToKey(minY);
    mBBXMin[2] = mOcTree.coordToKey(minZ);
    mBBXMax[0] = mOcTree.coordToKey(maxX);
    mBBXMax[1] = mOcTree.coordToKey(maxY);
    mBBXMax[2] = mOcTree.coordToKey(maxZ);
}

void
PCLServer::InsertCloud(const PCLPointCloud::Ptr aCloud, Pose& aCameraPose)
{
    PCLPointCloud Cloud;
    Eigen::Matrix4f worldPose;
    TransformAsMatrix(aCameraPose.pose_data, worldPose);
    pcl::transformPointCloud(*aCloud, *aCloud, worldPose);
    Cloud = *aCloud;
    Cloud.header = aCloud->header;    
    InsertScan(GetOriginVector(aCameraPose.pose_data), Cloud);
}

void
PCLServer::TransformAsMatrix(sl::Transform& aPoseData, Eigen::Matrix4f& aMPose)
{
    for (int i = 0; i <= 3; i++) {
        for (int j = 0; j <= 3; j++) {
            aMPose(i, j) = aPoseData(i, j);
        }
    }
}

point3d
PCLServer::GetOriginVector(Transform& aPoseData)
{
    return point3d(aPoseData(0, 3), aPoseData(1, 3), aPoseData(2, 3));
}

void
PCLServer::InsertScan(const point3d aOriginVector,
                      const PCLPointCloud& aCloud)
{
    if (!mOcTree.coordToKeyChecked(aOriginVector, mBBXMin) ||
        !mOcTree.coordToKeyChecked(aOriginVector, mBBXMax)) {
        cerr << "Could not generate Key for origin\n";
    }
    unsigned char* colors = new unsigned char[3];
    KeySet freeCells, occupiedCells;
    for (auto &it : aCloud.points) {
        point3d point(it.x, it.y, it.z);
        if ((mMaxRange < 0.0) || ((point - aOriginVector).norm() <= mMaxRange)) {
            if (mOcTree.computeRayKeys(aOriginVector, point, mKeyRay)) {
                freeCells.insert(mKeyRay.begin(), mKeyRay.end());
            }
            OcTreeKey key;
            if (mOcTree.coordToKeyChecked(point, key))
                occupiedCells.insert(key);
        } else {
            point3d newEnd = aOriginVector + (point - aOriginVector).normalized() * mMaxRange;
            if (mOcTree.computeRayKeys(aOriginVector, newEnd, mKeyRay)) {
                freeCells.insert(mKeyRay.begin(), mKeyRay.end());
                OcTreeKey endKey;
                if (mOcTree.coordToKeyChecked(newEnd, endKey)) {
                    freeCells.insert(endKey);
                    sUpdateKey(endKey, mBBXMin, mBBXMax);
                } else {
                    cerr << "Could not generate Key for endpoint\n";
                }
            }
        }
    }
    for (auto &it : freeCells) {
        if (occupiedCells.find(it) == occupiedCells.end()) {
            mOcTree.updateNode(it, false);
        }
    }
    for (auto &it : occupiedCells) {
        mOcTree.updateNode(it, true);
    }
    mOcTree.prune();
}

void
PCLServer::sUpdateKey(const OcTreeKey& aIn, OcTreeKey& aMin, OcTreeKey& aMax)
{
    for (int i = 0; i < 3; i++) {
        aMin[i] = min(aIn[i], aMin[i]);
        aMax[i] = max(aIn[i], aMax[i]);
    }
}

}

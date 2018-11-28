#include "pcl_server.h"

using boost::asio::ip::tcp;
using namespace sl;
using namespace std;

namespace server {

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

    StartZED();

    boost::asio::io_service ioService;
    tcp::endpoint endPoint(tcp::v4(), static_cast<unsigned short> (mPort));
    tcp::acceptor acceptor(ioService, endPoint);
    tcp::socket socket (ioService);

    cout << "Listening on port " << mPort << "...\n";
    acceptor.accept(socket);
    cout << "Client connected.\n";
    while (!mViewer.wasStopped()) {
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
            mVoxelGridFilter.setInputCloud(p_pcl_point_cloud);
            mVoxelGridFilter.filter(*point_cloud_buff);
            point_cloud_buff = GetRegSeg(&point_cloud_buff);
            float4 pE = mPlane.getPlaneEquation();
            for (auto& it : p_pcl_point_cloud->points)
                if ((pE[0] * it.x + pE[1] * it.y + pE[2] * it.z) == eq[3])
                    it.rgb = ConvertColor(0xffffff);
            unsigned int nrPoints = 
                static_cast<unsigned int>(point_cloud_buff->points.size());
            boost::asio::write(socket,
                boost::asio::buffer(&nrPoints, sizeof(nrPoints)));
            boost::asio::write(socket,
                boost::asio::buffer(&point_cloud_buff->points.front(),
                    nrPoints * 8 * sizeof(float)));
            mMutexInput.unlock();
            mViewer.showCloud(point_cloud_buff);
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
            Transform resetTrakingFloorFrame;
            mZed.findFloorPlane(mPlane, resetTrakingFloorFrame);
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
PCLServer::GetRegSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *aPointCloud)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = 
        boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>>
            (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(*aPointCloud);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(5);
    reg.setPointColorThreshold(3);
    reg.setRegionColorThreshold(2);
    reg.setMinClusterSize(60);
    vector <pcl::PointIndices> clusters;
    reg.extract(clusters);
    return reg.getColoredCloud();
}

}

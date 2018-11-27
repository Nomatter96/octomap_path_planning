#include "pcl_server.h"

using boost::asio::ip::tcp;
using namespace sl;
using namespace std;

namespace server {

void 
PCLServer::Run()
{
    InitParameters sInitParams;
    sInitParams.camera_resolution = RESOLUTION_VGA;
    sInitParams.camera_fps = 30;
    sInitParams.coordinate_units = UNIT_METER;
    sInitParams.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    sInitParams.depth_mode = DEPTH_MODE_PERFORMANCE;
	
    ERROR_CODE sErr = mZed.open(sInitParams);
    if (sErr != SUCCESS) {
        cout << toString(sErr) << "\n";
        mZed.close();
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        sPPclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sPPclPointCloud->points.resize(mZed.getResolution().area());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
        sPointCloudBuff(new pcl::PointCloud<pcl::PointXYZRGB>);

    StartZED();

    boost::asio::io_service sIoService;
    tcp::endpoint sEndpoint(tcp::v4(), static_cast<unsigned short> (mPort));
    tcp::acceptor sAcceptor(sIoService, sEndpoint);
    tcp::socket sSocket (sIoService);

    cout << "Listening on port " << mPort << "..." << "\n";
    sAcceptor.accept(sSocket);
    cout << "Client connected." << "\n";
    mSizeBuff = mDataCloud.getHeight() * mDataCloud.getWidth() * 4;
    while (!mViewer.wasStopped()) {
        if (mMutexInput.try_lock()) {
            float *sPDataCloud = mDataCloud.getPtr<float>();
            int sIndex = 0;
            for (auto &it : sPPclPointCloud->points) {
                if (!isValidMeasure(sPDataCloud[sIndex]))
                    it.x = it.y = it.z = it.rgb = 0;  
                else {
                    it.x = sPDataCloud[sIndex];
                    it.y = sPDataCloud[sIndex + 1];
                    it.z = sPDataCloud[sIndex + 2];
                    it.rgb = ConvertColor(sPDataCloud[sIndex + 3]);
                }
                sIndex += 4;
            }
            mVoxelGridFilter.setInputCloud(sPPclPointCloud);
            mVoxelGridFilter.filter(*sPointCloudBuff);
            sPointCloudBuff = GetRegSeg(&sPointCloudBuff);
            unsigned int sNrPoints = 
                static_cast<unsigned int>(sPointCloudBuff->points.size());
            boost::asio::write(sSocket,
                boost::asio::buffer(&sNrPoints, sizeof(sNrPoints)));
            boost::asio::write(sSocket,
                boost::asio::buffer(&sPointCloudBuff->points.front(),
                    sNrPoints * 8 * sizeof(float)));
            mMutexInput.unlock();
            mViewer.showCloud(sPointCloudBuff);
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
    uint32_t sColorUint = *(uint32_t *) & aColorIn;
    unsigned char *sColorUchar = (unsigned char *) &sColorUint;
    sColorUint = ((uint32_t) sColorUchar[0] << 16 |
        (uint32_t) sColorUchar[1] << 8 | (uint32_t) sColorUchar[2]);
    return *reinterpret_cast<float *> (&sColorUint);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
PCLServer::GetRegSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *aPointCloud)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr sTree = 
        boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>>
            (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> sReg;
    sReg.setInputCloud(*aPointCloud);
    sReg.setSearchMethod(sTree);
    sReg.setDistanceThreshold(5);
    sReg.setPointColorThreshold(3);
    sReg.setRegionColorThreshold(2);
    sReg.setMinClusterSize(60);
    vector <pcl::PointIndices> sClusters;
    sReg.extract(sClusters);
    return sReg.getColoredCloud();
}

}

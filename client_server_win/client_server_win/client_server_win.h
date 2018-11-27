#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/config.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using boost::asio::ip::tcp;

namespace cl {

class PCLClient
{
private:
    boost::asio::io_service& mIoService;
    tcp::socket mSocket;
    unsigned int mNrPoints;
    pcl::visualization::CloudViewer mViewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mBuf;

    void HandleConnect(const boost::system::error_code& aError);
    void DoClose() { mSocket.close(); };

public:
    PCLClient(boost::asio::io_service& aIoService,
        tcp::resolver::iterator aEndpointIterator)
        : mIoService(aIoService) 
        , mSocket(aIoService)
        , mViewer("PCL") 
    {
        boost::asio::async_connect(mSocket, aEndpointIterator, 
            boost::bind(&PCLClient::HandleConnect, this, 
                boost::asio::placeholders::error));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sBuf(new pcl::PointCloud<pcl::PointXYZRGB>);
        mBuf = sBuf;
    }
    ~PCLClient() {};
};

}
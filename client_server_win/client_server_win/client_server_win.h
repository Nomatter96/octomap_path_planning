#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/system/config.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

using boost::asio::ip::tcp;

class PCLClient 
{
	private:
		boost::asio::io_service& io_service_;
		tcp::socket socket_;
		unsigned int nr_points = 0;
		pcl::visualization::CloudViewer viewer_;
		
		void handle_connect(const boost::system::error_code& error);
		void do_close() { socket_.close(); };

	public:
		PCLClient(boost::asio::io_service& io_service, tcp::resolver::iterator endpoint_iterator) : io_service_(io_service), socket_(io_service), viewer_("PCL") {
			boost::asio::async_connect(socket_, endpoint_iterator, boost::bind(&PCLClient::handle_connect, this, boost::asio::placeholders::error));
		}
		~PCLClient() { close(); };
		void close() { io_service_.post(boost::bind(&PCLClient::do_close, this)); };
};

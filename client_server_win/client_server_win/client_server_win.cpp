#include "client_server_win.h"

using namespace std;


void PCLClient::handle_connect(const boost::system::error_code& error) {
	if (error)
		return;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr buf(new pcl::PointCloud<pcl::PointXYZRGB>);
	while (!viewer_.wasStopped()) {
		boost::asio::read(socket_, boost::asio::buffer(&nr_points, sizeof(nr_points)));
		if (nr_points != 0) {
			buf->points.resize(nr_points);
			boost::asio::read(socket_, boost::asio::buffer(&buf->points.front(), nr_points * 8 * sizeof(float)));
			viewer_.showCloud(buf);
		}
		nr_points = 0;
	}
};
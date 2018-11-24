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
			boost::asio::read(socket_, boost::asio::buffer(&buf->points.front(), nr_points * sizeof(float)));
			viewer_.showCloud(buf);
		}
		nr_points = 0;
	}
};

shared_ptr<pcl::visualization::PCLVisualizer> PCLClient::createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
	viewer->setBackgroundColor(0.12, 0.12, 0.12);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
};
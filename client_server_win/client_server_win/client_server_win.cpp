#include "client_server_win.h"

using namespace std;


void PCLClient::handle_connect(const boost::system::error_code& error) {
	if (error)
		return;
	while (!viewer_.wasStopped()) {
		boost::asio::read(socket_, boost::asio::buffer(&nr_points, sizeof(nr_points)));
		if (nr_points != 0) {
			buf_->points.resize(nr_points);
			boost::asio::read(socket_, boost::asio::buffer(&buf_->points.front(), nr_points * 8 * sizeof(float)));
			viewer_.showCloud(getRegSeg());
		}
		nr_points = 0;
	}
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLClient::getRegSeg() {
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);
	/*pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);*/

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(buf_);
	//reg.setIndices(indices);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(10);
	reg.setPointColorThreshold(6);
	reg.setRegionColorThreshold(5);
	reg.setMinClusterSize(600);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	return reg.getColoredCloud();
}
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
			viewer_.showCloud(buf_);
		}
		nr_points = 0;
	}
};

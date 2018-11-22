#include "client_server_win.h"

using namespace std;


void PCLClient::handle_connect(const boost::system::error_code& error) {
	if (error)
		return;
	//cout << buf[1];
	boost::asio::async_read(socket_, boost::asio::buffer(buf), boost::bind(&PCLClient::handle_connect, this, boost::asio::placeholders::error));
};

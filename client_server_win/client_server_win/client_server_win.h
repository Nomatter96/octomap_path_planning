#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

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
		std::vector<short> buf;
		void handle_connect(const boost::system::error_code& error);
		void do_close() { socket_.close(); };

	public:
		PCLClient(boost::asio::io_service& io_service, tcp::resolver::iterator endpoint_iterator) : io_service_(io_service), socket_(io_service) {
			boost::asio::async_connect(socket_, endpoint_iterator, boost::bind(&PCLClient::handle_connect, this, boost::asio::placeholders::error));
		}
		~PCLClient() { close(); };
		//char* getBuff() { return buf; };
		void close() { io_service_.post(boost::bind(&PCLClient::do_close, this)); };
};

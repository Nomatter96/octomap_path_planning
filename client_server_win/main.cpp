#include "client_server_win/client_server_win.h"

using namespace std;

int main(int argc, char* argv[]) {
	try
	{
		if (argc != 3) {
			std::cerr << "Usage: <host> <port>\n";
			return -1;
		}
		boost::asio::io_service io_service;
		tcp::resolver resolver(io_service);
		tcp::resolver::query query(argv[1], argv[2]);
		tcp::resolver::iterator iterator = resolver.resolve(query);
		PCLClient client(io_service, iterator);
		io_service.run();
	}
	catch (exception& e) {
		cerr << "Exception: " << e.what() << "\n";
	}
}
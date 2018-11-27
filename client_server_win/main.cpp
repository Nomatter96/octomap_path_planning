#include "client_server_win/client_server_win.h"

using namespace std;

int
main(int argc, char* argv[]) 
{
    try
    {
        if (argc != 3) {
            std::cerr << "Usage: <host> <port>\n";
            return -1;
        }
        boost::asio::io_service sIoService;
        tcp::resolver sResolver(sIoService);
        tcp::resolver::query sQuery(argv[1], argv[2]);
        tcp::resolver::iterator sIterator = sResolver.resolve(sQuery);
        cl::PCLClient sClient(sIoService, sIterator);
        sIoService.run();
    }
    catch (exception& aErr) {
        cerr << "Exception: " << aErr.what() << "\n";
    }
}

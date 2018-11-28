#include "pcl_server/pcl_server.h"

#include <stdio.h>
#include <string.h>

using namespace std;

void
usage (char **argv)
{
    cout << "usage: " << argv[0] << " <options>\n"
        << "where options are:\n"
        << "  -port p :: set the server port (default: 11111)\n"
        << "  -leaf x, y, z  :: set the voxel grid leaf size (default: 0.01)\n";
}

int
main (int argc, char ** argv)
{
    if (pcl::console::find_argument(argc, argv, "-h") != -1) {
        usage(argv);
        return 0;
    }
	
    int port = 11111;
    float leafX = 0.03f, leafY = 0.03f, leafZ = 0.03f;

    pcl::console::parse_argument(argc, argv, "-port", port);		
    pcl::console::parse_3x_arguments(argc, argv, "-leaf", leafX, leafY, leafZ, false);

    server::PCLServer server(port, leafX, leafY, leafZ);
    server.Run();
    return 0;
}

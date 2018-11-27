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
	
    int sPort = 11111;
    float sLeafX = 0.03f, sLeafY = 0.03f, sLeafZ = 0.03f;

    pcl::console::parse_argument(argc, argv, "-port", sPort);		
    pcl::console::parse_3x_arguments(argc, argv, "-leaf", sLeafX, sLeafY, sLeafZ, false);

    server::PCLServer sServer(sPort, sLeafX, sLeafY, sLeafZ);
    sServer.Run();
    return 0;
}

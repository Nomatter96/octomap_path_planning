#include "pcl_server/pcl_server.h"

#include <stdio.h>
#include <string.h>

using namespace std;

int
main (int argc, char ** argv)
{
    int port = 11111;
    float leafX = 0.05f, leafY = 0.05f, leafZ = 0.05f;

    pcl::console::parse_argument(argc, argv, "-port", port);		
    pcl::console::parse_3x_arguments(argc, argv, "-leaf", leafX, leafY, leafZ, false);
    server::PCLServer server(port, leafX, leafY, leafZ);
    server.Run();
    return 0;
}

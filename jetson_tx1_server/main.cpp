#include <stdio.h>
#include <string.h>
#include "pcl_server/pcl_server.h"

using namespace sl;

void usage (char **argv) {
	std::cout << "usage: " << argv[0] << " <options>\n"
		<< "where options are:\n"
		<< "  -port p :: set the server port (default: 11111)\n"
		<< "  -leaf x, y, z  :: set the voxel grid leaf size (default: 0.01)\n";
}

int main (int argc, char ** argv) {
	if (pcl::console::find_argument (argc, argv, "-h") != -1){
    		usage (argv);
    		return (0);
  	}
	
	int port = 11111;
	float leaf_x = 0.05f, leaf_y = 0.05f, leaf_z = 0.05f;

	pcl::console::parse_argument(argc, argv, "-port", port);		
	pcl::console::parse_3x_arguments(argc, argv, "-leaf", leaf_x, leaf_y, leaf_z, false);
	
	PCLServer server(port, leaf_x, leaf_y, leaf_z);
	server.run();
	return (0);
}

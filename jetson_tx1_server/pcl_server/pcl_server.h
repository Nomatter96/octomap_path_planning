#pragma once

#ifdef _WIN32
#undef max
#undef min
#endif

#include <thread>
#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <sl_zed/Camera.hpp>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>


class PCLServer
{
	public:
		PCLServer(const int port = 11111, const float leaf_size_x = 0.01f, const float leaf_size_y = 0.01f, const float leaf_size_z = 0.01f) : port_(port), viewer_("PCL") {
			voxel_grid_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);			
		};
		~PCLServer() {};	
		void run();
		
	private:
		sl::Camera zed;
		sl::Mat data_cloud;
		bool stop_signal;
		bool has_data;
		int port_;
		std::thread zed_callback;
		std::mutex mutex_input;
		pcl::visualization::CloudViewer viewer_;		
		long long int sizebuf;
		pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter_;

		void startZED();
		void start();
		void closeZED();
		inline float convertColor(float colorIn);
};

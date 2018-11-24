#include "pcl_server.h"

using namespace std;
using namespace sl;
using boost::asio::ip::tcp;

void PCLServer::run() {
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_VGA;
	init_params.camera_fps = 30;
	init_params.coordinate_units = UNIT_METER;
	init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	
	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS) {
		cout << toString(err) << endl;
		zed.close();
		return;
	}
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	p_pcl_point_cloud->points.resize(zed.getResolution().area());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buff(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//viewer_ = createRGBVisualizer(p_pcl_point_cloud);
	
	startZED();
	
	boost::asio::io_service io_service;
	tcp::endpoint endpoint(tcp::v4(), static_cast<unsigned short> (port_));
	tcp::acceptor acceptor(io_service, endpoint);
	tcp::socket socket (io_service);
		
	cout << "Listening on port " << port_ << "..." << endl;
	acceptor.accept(socket);
	cout << "Client connected." << endl;
	unsigned int nr_points = 0;
	sizebuf = data_cloud.getHeight() * data_cloud.getWidth() * 4;
	while (!viewer_.wasStopped()) {
		if (mutex_input.try_lock()) {
			float *p_data_cloud = data_cloud.getPtr<float>();
			int index = 0;
			for (auto &it : p_pcl_point_cloud->points) {
				if (!isValidMeasure(p_data_cloud[index]))
					it.x = it.y = it.z = it.rgb = 0;  
				else {
					it.x = p_data_cloud[index];
					it.y = p_data_cloud[index + 1];
					it.z = p_data_cloud[index + 2];
					it.rgb = convertColor(p_data_cloud[index + 3]);
				}
				index += 4;
			}
			voxel_grid_filter_.setInputCloud(p_pcl_point_cloud);
			voxel_grid_filter_.filter(*point_cloud_buff);
			nr_points = static_cast<unsigned int>(point_cloud_buff->points.size());
			boost::asio::write(socket, boost::asio::buffer(&nr_points, sizeof(nr_points)));
			boost::asio::write(socket, boost::asio::buffer(&point_cloud_buff->points.front(), nr_points * sizeof(float)));
			mutex_input.unlock();
			viewer_.showCloud(point_cloud_buff);
			//viewer_->updatePointCloud(point_cloud_buff);
			//viewer_->spinOnce(10);
		}
		else
			sleep_ms(1);
	}
	//viewer_->close();
	closeZED();
}

void PCLServer::startZED() {
	stop_signal = false;
	has_data = false;
	zed_callback = thread(&PCLServer::start, this);
	while (!has_data)
		sleep_ms(1);
}

void PCLServer::start() {
	while (!stop_signal) {
		if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
			mutex_input.lock();
			zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);
			mutex_input.unlock();
			has_data = true;
		}
		else
			sleep_ms(1);
	}
}

void PCLServer::closeZED() {
	stop_signal = true;
	zed_callback.join();
	zed.close();
}

void PCLServer::CopyPointCloudToBuffers(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, PointCloudBuffers& cloud_buffers) {
	cout << "ok" << endl;	
	const size_t nr_points = cloud->points.size();
	cloud_buffers.points.resize(nr_points * 3);
	cloud_buffers.rgb.resize(nr_points * 3);
	size_t j = 0;
	for (size_t i = 0; i < nr_points; i++) {
		const pcl::PointXYZRGB& point = cloud->points[i];
		if (!pcl_isfinite(point.x) || !pcl_isfinite(point.y) || !pcl_isfinite(point.z))
			continue;
		const int conversion_factor = 500;
		cloud_buffers.points[j * 3] = static_cast<short>(point.x * conversion_factor);
		cloud_buffers.points[j * 3 + 1] = static_cast<short>(point.y * conversion_factor);
		cloud_buffers.points[j * 3 + 2] = static_cast<short>(point.z * conversion_factor);
		cloud_buffers.rgb[j * 3] = point.r;
		cloud_buffers.rgb[j * 3 + 1] = point.g;
		cloud_buffers.rgb[j * 3 + 2] = point.b;
		j++;
	}
	cloud_buffers.points.resize(j * 3);
	cloud_buffers.rgb.resize(j * 3);
}

shared_ptr<pcl::visualization::PCLVisualizer> PCLServer::createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

inline float PCLServer::convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}

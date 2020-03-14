/*
 * Created by Maou Lim on 2020/3/14.
 */

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <memory>

#include <camera.hpp>
#include <odometry.hpp>
#include <config.hpp>


const std::string conf_file = "config/default.yaml";

int main(int argc, char* argv[]) {

	sslam::config::load_configuration(conf_file);

	const std::string dataset_dir =
		sslam::config::get<std::string>("dataset_dir");

	std::ifstream stream(dataset_dir + "/associate.txt");
	if (!stream.good()) {
		std::cerr << "Failed to open associate.txt" << std::endl;
		return -1;
	}

	std::vector<std::string> rgb_files     , depth_files;
	std::vector<double>      rgb_timestamps, depth_timestamps;

	while (!stream.eof()) {
		std::string rgb_timestamp  , rgb_file ,
		            depth_timestamp, depth_file;

		stream >> rgb_timestamp >> rgb_file >> depth_timestamp >> depth_file;
		rgb_timestamps.push_back(atof(rgb_timestamp.c_str()));
		depth_timestamps.push_back(atof(depth_timestamp.c_str()));
		rgb_files.push_back(dataset_dir + "/" + rgb_file);
		depth_files.push_back(dataset_dir + "/" + depth_file);
	}

	auto camera = sslam::camera::create();

	cv::viz::Viz3d visualizer()

	auto vo = sslam::odometry::create();


	return 0;
}










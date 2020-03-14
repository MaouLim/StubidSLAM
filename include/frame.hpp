/*
 * Created by Maou Lim on 2020/3/13.
 */

#ifndef _STUBIDSLAM_FRAME_HPP_
#define _STUBIDSLAM_FRAME_HPP_

#include <camera.hpp>
#include <memory>
#include <opencv2/opencv.hpp>

namespace sslam {

	class frame {
	public:
		using camera_type = camera;
		using ptr         = std::shared_ptr<frame>;

		frame() = default;
		frame(
			uint64_t     id,
			double       timestamp = 0.0,
			camera::ptr  cam   = nullptr,
			Sophus::SE3d pose  = Sophus::SE3d(),
			cv::Mat      color = cv::Mat(),
			cv::Mat      depth = cv::Mat()
		) : _id(id), _timestamp(timestamp), _camera(cam),
		    _pose(pose), _color(color), _depth(depth) { }
		~frame() = default;

		/* getters */
		uint64_t id() const { return _id; }
		const Sophus::SE3d& pose() const { return _pose; }
		const camera::ptr& camera() const { return _camera; }
		const cv::Mat& color_image() const { return _color; }
		const cv::Mat& depth_image() const { return _depth; }

		/* modify-getters */
		Sophus::SE3d& pose() { return _pose; }
		cv::Mat& color_image() { return _color; }
		cv::Mat& depth_image() { return _depth; }

		double depth_at(double x, double y) const {
			int _x = cvRound(x);
			int _y = cvRound(y);
			// todo 检查是否越界
			using depth_t = typename camera_type::depth_type;
			depth_t d = _depth.at<depth_t>(_y, _x);
			if (0 != d) { return double(d) * _camera->depth_scale(); }
			else {
				// todo 优化为窗口检查
				int dx[4] = { -1, 0,  1, 0 };
				int dy[4] = {  0, -1, 0, 1 };
				for (auto i = 0; i < 4; ++i) {
					d = _depth.at<depth_t>(_y + dy[i], _x + dx[i]);
					if (0 != d) { return double(d) * _camera->depth_scale(); }
				}
			}
			return -1.;
		}

		Eigen::Vector3d camera_center() const { return _pose.inverse().translation(); }

		bool in_frame(const Eigen::Vector3d& p_world) const {
			Eigen::Vector3d p_cam = camera::from_world(p_world, _pose);
			if (p_cam[3] < 0) { return false; } // 在相机的背后
			Eigen::Vector2d p_pixel = _camera->camera2pixel(p_cam);
			return         0. <= p_pixel[0]  &&         0. <= p_pixel[1]  &&
			       p_pixel[0] <  _color.cols && p_pixel[1] <  _color.rows;
		}

		static ptr create() {
			// todo 线程安全
			static uint64_t seq_id = 0;
			return std::make_shared<frame>(seq_id++);
		}

	private:
		uint64_t     _id;
		double       _timestamp;
		camera::ptr  _camera;
		Sophus::SE3d _pose;
		cv::Mat      _color, _depth;
	};
}

#endif //STUBIDSLAM_FRAME_HPP

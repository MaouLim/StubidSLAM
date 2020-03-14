/*
 * Created by Maou Lim on 2020/3/13.
 */

#ifndef _STUBIDSLAM_CAMERA_HPP_
#define _STUBIDSLAM_CAMERA_HPP_

#include <memory>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <config.hpp>

namespace sslam {

	class camera {
	public:
		using ptr        = std::shared_ptr<camera>;
		using depth_type = uint16_t;

		camera() {
			_fx = config::get<double>("camera.fx");
			_fy = config::get<double>("camera.fy");
			_cx = config::get<double>("camera.cx");
			_cy = config::get<double>("camera.cy");
			_depth_scale = config::get<double>("camera.depth_scale");
		}

		camera(
			double fx, double fy,
			double cx, double cy,
			double depth_scale = 0.
		) : _fx(fx), _fy(fy),
		    _cx(cx), _cy(cy),
		    _depth_scale(depth_scale) { }

		static Eigen::Vector3d
			from_world(const Eigen::Vector3d& p_world, const Sophus::SE3d& cam_pose)
		{ return cam_pose * p_world; }

		static Eigen::Vector3d
			to_world(const Eigen::Vector3d& p_cam, const Sophus::SE3d& cam_pose)
		{ return cam_pose.inverse() * p_cam; }

		Eigen::Vector2d camera2pixel(const Eigen::Vector3d& p_cam) const {
			return { _fx * p_cam[0] / p_cam[2] + _cx, _fy * p_cam[1] / p_cam[2] + _cy };
		}

		Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p_pixel, double depth = 1.) const {
			return { (p_pixel[0] - _cx) * depth / _fx, (p_pixel[1] - _cy) * depth / _fy, depth };
		}

		Eigen::Vector2d
			world2pixel(const Eigen::Vector3d& p_world, const Sophus::SE3d& cam_pose) const
		{ return camera2pixel(from_world(p_world, cam_pose)); }

		Eigen::Vector3d
			pixel2world(const Eigen::Vector2d& p_pixel, const Sophus::SE3d& cam_pose, double depth = 1.) const
		{ return to_world(pixel2camera(p_pixel, depth), cam_pose); }

		cv::Mat cv_mat() const { return (cv::Mat_<double>(3, 3) << _fx, 0., _cx, 0., _fy, _cy, 0., 0., 1.); }

		Eigen::Matrix3d eigen_mat() const {
			Eigen::Matrix3d mat;
			mat << _fx, 0., _cx, 0., _fy, _cy, 0., 0., 1.;
			return mat;
		}

		double depth_scale() const { return _depth_scale; }
		double& depth_scale() { return _depth_scale; }

		static ptr create() { return std::make_shared<camera>(); }

	private:
		/* 相机内参 */
		double _fx, _fy;
		double _cx, _cy;
		double _depth_scale;
	};
}

#endif //STUBIDSLAM_CAMERA_HPP

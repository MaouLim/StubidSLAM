/*
 * Created by Maou Lim on 2020/3/14.
 */

#ifndef _STUBIDSLAM_ODOMETRY_HPP_
#define _STUBIDSLAM_ODOMETRY_HPP_

#include <memory>
#include <vector>

namespace sslam {

	class odometry {
	public:
		using ptr = std::shared_ptr<odometry>;

		enum state { INITIALING, RUNNING, LOST };

		odometry() :
			_state(INITIALING), _prv(nullptr), _cur(nullptr),
			_map(map::create()), _count_lost(0), _count_inliers(0)
		{
			_n_features                = config::get<int>    ("n_features");
			_scale_factor              = config::get<double> ("scale_factor");
			_level_pyramid             = config::get<int>    ("level_pyramid");
			_match_ratio               = config::get<double> ( "match_ratio" );
			_max_lost                  = config::get<int>    ("max_lost");
			_min_inliers               = config::get<int>    ("min_inliers");
			_key_frame_min_rotation    = config::get<double> ("key_frame_min_rotation");
			_key_frame_min_translation = config::get<double> ("key_frame_min_translation");
			_orb_detector = cv::ORB::create(_n_features, _scale_factor, _level_pyramid);
		}

		~odometry() = default;

		bool feed_frame(frame::ptr frame) {
			switch (_state) {
				case INITIALING : {
					_state = RUNNING;
					_cur = _prv = frame;
					_map->insert_or_replace(frame);

					this->_extract_kpts();
					this->_compute_descriptors();
					this->_initialize_3dpts();

					break;
				}
				case RUNNING : {
					_cur = frame;

					this->_extract_kpts();
					this->_compute_descriptors();
					this->_match();
					this->_pose_estimation_pnp();

					if (!_check_estimation()) {
						++_count_lost;
						if (_max_lost < _count_lost) {
							_state = LOST;
						}
						return false;
					}

					_cur->pose() = _pose * _prv->pose();
					_prv = _cur;
					this->_initialize_3dpts();
					_count_lost = 0;
					if (_is_key_frame()) {
						std::cout << "Key frame detected." << std::endl;
						_map->insert_or_replace(_cur);
					}
					break;
				}
				case LOST : {
					std::cerr << "VO lost." << std::endl;
					return false;
				}
				default : { return false; }
			}
			return true;
		}

		static ptr create() { return std::make_shared<odometry>(); }

	private:
		void _extract_kpts() { _orb_detector->detect(_cur->color_image(), _kpts_cur); }

		void _compute_descriptors() { _orb_detector->compute(_cur->color_image(), _kpts_cur, _descriptor_cur); }

		void _match() {
			std::vector<cv::DMatch> tmp;
			cv::BFMatcher matcher(cv::NORM_HAMMING);
			matcher.match(_descriptor_prv, _descriptor_cur, tmp);

			float min_dis = std::min_element(
				tmp.begin(), tmp.end(),
				[](const cv::DMatch& ma, const cv::DMatch& mb) { return ma.distance < mb.distance; }
			)->distance;

			_matches.clear();
			for (auto& each : tmp) {
				if (each.distance < std::max(min_dis * _match_ratio, 30.)) {
					_matches.push_back(each);
				}
			}
		}

		void _initialize_3dpts() {
			_3dpts_prv.clear();
			_descriptor_prv = cv::Mat();

			for (size_t i = 0; i < _kpts_cur.size(); ++i) {
				auto& kpt = _kpts_cur[i].pt;
				double depth = _prv->depth_at(kpt.x, kpt.y);
				if (0. < depth) {
					auto p_cam = _prv->camera()->pixel2camera({ kpt.x, kpt.y }, depth);
					_3dpts_prv.emplace_back(p_cam.x(), p_cam.y(), p_cam.z());
					_descriptor_prv.push_back(_descriptor_cur.row(i));
				}
			}
		}

		void _pose_estimation_pnp() {
			std::vector<cv::Point3f> pts3d;
			std::vector<cv::Point2f> pts2d;

			for (auto& each : _matches) {
				pts3d.push_back(_3dpts_prv[each.queryIdx]);
				pts2d.push_back(_kpts_cur[each.trainIdx].pt);
			}

			cv::Mat camera_mat = _prv->camera()->cv_mat();
			cv::Mat rvec, tvec, inliers;

			cv::solvePnPRansac(pts3d, pts2d, camera_mat, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
			_count_inliers = inliers.rows;

			std::cout << "PNP inliers: " << _count_inliers << std::endl;

			Sophus::SO3d so3 =
				Sophus::SO3d::exp({ rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2) });
			Eigen::Vector3d t =
				{ tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2) };
			_pose = Sophus::SE3d(so3, t);
		}

		bool _check_estimation() const {
			if (_count_inliers < _min_inliers) {
				std::cerr
					<< "VO rejects the estimation result since inliers are too small."
					<< std::endl;
				return false;
			}

			auto tangent = _pose.log();
			double length = tangent.norm();
			if (5.0 < length) {
				std::cerr
					<< "VO rejects the estimation result since the motion between two frames is too large: "
					<< length
					<< std::endl;
				return false;
			}
			return true;
		}

		bool _is_key_frame() const {
			auto tangent = _pose.log();
			Eigen::Vector3d t = tangent.head<3>();
			Eigen::Vector3d r = tangent.tail<3>();
			return _key_frame_min_rotation < r.norm() || _key_frame_min_translation < t.norm();
		}

	private:
		state        _state;
		map::ptr     _map;
		frame::ptr   _prv, _cur;
		Sophus::SE3d _pose;

		/* OpenCV feature extractor stuff */
		cv::Ptr<cv::ORB>          _orb_detector;
		std::vector<cv::KeyPoint> _kpts_cur;
		std::vector<cv::Point3f>  _3dpts_prv;

		cv::Mat                 _descriptor_prv;
		cv::Mat                 _descriptor_cur;
		std::vector<cv::DMatch> _matches;

		uint64_t _count_inliers;
		uint64_t _count_lost;

		/* visual odometry base parameters */
		uint64_t _n_features;
		uint64_t _max_lost;
		uint64_t _min_inliers;
		uint64_t _level_pyramid;
		double   _scale_factor;
		double   _match_ratio;
		double   _key_frame_min_rotation;
		double   _key_frame_min_translation;
	};
}

#endif //STUBIDSLAM_ODOMETRY_HPP

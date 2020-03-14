/*
 * Created by Maou Lim on 2020/3/13.
 */

#ifndef _STUBIDSLAM_LANDMARK_HPP_
#define _STUBIDSLAM_LANDMARK_HPP_

namespace sslam {

	class landmark {
	public:
		using ptr = std::shared_ptr<landmark>;

		landmark() = default;
		landmark(
			uint64_t               id,
			const Eigen::Vector3d& pos        = { 0., 0., 0. },
			const Eigen::Vector3d& view_orien = { 1., 0., 0. }
		) : _id(id), _position(pos), _view_orien(view_orien),
		    _descriptor(cv::Mat()), _observed_times(0), _corrected_times(0) { }

		uint64_t id() const { return _id; }

		static ptr create() {
			static uint64_t seq_id = 0;
			return std::make_shared<landmark>(seq_id++);
		}

	private:
		uint64_t        _id;
		Eigen::Vector3d _position;
		Eigen::Vector3d _view_orien;
		cv::Mat         _descriptor;
		uint64_t        _observed_times;
		uint64_t        _corrected_times;
	};

}

#endif //STUBIDSLAM_LANDMARK_HPP

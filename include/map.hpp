/*
 * Created by Maou Lim on 2020/3/13.
 */

#ifndef _STUBIDSLAM_MAP_HPP_
#define _STUBIDSLAM_MAP_HPP_

#include <unordered_map>

namespace sslam {

	class map {
	public:
		using ptr = std::shared_ptr<map>;

		map() = default;

		void insert_or_replace(landmark::ptr landmark) {
			auto hint = _landmarks.find(landmark->id());
			if (_landmarks.end() == hint) {
				_landmarks.insert(hint, std::make_pair(landmark->id(), landmark));
			}
			else { hint->second = landmark; }
		}

		void insert_or_replace(frame::ptr frame) {
			auto hint = _key_frames.find(frame->id());
			if (_key_frames.end() == hint) {
				_key_frames.insert(hint, std::make_pair(frame->id(), frame));
			}
			else { hint->second = frame; }
		}

		static ptr create() { return std::make_shared<map>(); }

	private:
		std::unordered_map<uint64_t, landmark::ptr> _landmarks;
		std::unordered_map<uint64_t, frame::ptr>    _key_frames;
	};
}

#endif //STUBIDSLAM_MAP_HPP

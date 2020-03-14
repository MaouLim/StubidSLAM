/*
 * Created by Maou Lim on 2020/3/13.
 */

#ifndef _STUBIDSLAM_CONFIG_HPP_
#define _STUBIDSLAM_CONFIG_HPP_

namespace sslam {

	class config {
	public:
		using ptr = std::shared_ptr<config>;

		template <typename _Parameter>
		static _Parameter get(const std::string& name) {
			return _Parameter(config::_global_conf->_storage[name]);
		}

		static void load_configuration(const std::string& path) {
			if (nullptr == _global_conf) {
				_global_conf = std::shared_ptr<config>(new config);
			}
			_global_conf->_storage = cv::FileStorage(path, cv::FileStorage::READ);
			if (!_global_conf->_storage.isOpened()) {
				std::cerr << "Failed to open file: " << path << std::endl;
				_global_conf->_storage.release();
				_global_conf.reset();
			}
		}

	private:
		static config::ptr _global_conf;

		cv::FileStorage    _storage;

		config() = default;
	};

	config::ptr config::_global_conf(nullptr);
}

#endif //STUBIDSLAM_CONFIG_HPP

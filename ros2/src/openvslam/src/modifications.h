system.h
void save_json_file(const std::string& path) const;


system.cc

void system::save_json_file(const std::string& path) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_json_file(path);
    resume_other_threads();
}


trajectory_io.h

void save_json_file(const std::string& path) const;


trajectory_io.cc

void trajectory_io::save_json_file(const std::string& path) const {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);


    assert(map_db_);
    auto points = map_db_->get_all_landmarks();


    if (points.empty()) {
        spdlog::warn("there are no valid points, cannot dump points position");
        return;
    }

    std::ofstream ofs(path, std::ios::out);
    if (!ofs.is_open()) {
        spdlog::critical("cannot create a file at {}", path);
        throw std::runtime_error("cannot create a file at " + path);
    }

    for (const auto points : points) {
        nlohmann::json dict = points->to_json();
        ofs << dict;
    }

    ofs.close();
}

run_slam.cc

    SLAM.save_landmarks_and_timestamp("/home/mirellameelo/openvslam/ros2/maps/landmarks_and_timestamp.json");

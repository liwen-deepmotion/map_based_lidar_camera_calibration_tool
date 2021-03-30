// Author: Chi Zhang (chizhang@deepmotion.ai) 11/20/19

#include "lidar_trajectory.h"

#include <fstream>
#include <boost/format.hpp>

template <typename T>
void dump_str(const std::string& str, T& output);

LidarTrajectory::Ptr LidarTrajectory::New() {
    return std::make_shared<LidarTrajectory>();
}

void LidarTrajectory::LoadCsv(const std::string& csv_file_path) {
    std::ifstream file(csv_file_path, std::ios::in);
    LOG_IF(FATAL, !file.is_open()) << "Cannot open file: " << csv_file_path;

    std::string long_line;
    if (!std::getline(file, long_line)){
        return;
    }
    std::istringstream iss(long_line);

    std::string item;
    uint64_t count = 0;
    std::vector<std::vector<std::string>> lines;
    std::vector<std::string> line_params;
    while(std::getline(iss, item, ',')){
        if (count && count % 15 == 0){
            std::string prev_end = item.substr(0, 1);
            line_params.push_back(prev_end);
            lines.push_back(line_params);
            line_params.clear();
            // Remove the head 0.
            item = item.substr(1);
        }
        line_params.push_back(item);
        count++;
    }

    for (const auto& line : lines) {
        LidarFrame::Ptr frame = std::make_shared<LidarFrame>();
        dump_str<uint64_t>(line[0], frame->timestamp);

        int start_idx = 3;

        frame->T_self_to_enu = Eigen::Isometry3d::Identity();
        dump_str<double>(line[start_idx],
                frame->T_self_to_enu.translation()[0]);
        dump_str<double>(line[start_idx + 1],
                frame->T_self_to_enu.translation()[1]);
        dump_str<double>(line[start_idx + 2],
                frame->T_self_to_enu.translation()[2]);

        Eigen::Quaterniond quat;
        dump_str<double>(line[start_idx + 3], quat.w());
        dump_str<double>(line[start_idx + 4], quat.x());
        dump_str<double>(line[start_idx + 5], quat.y());
        dump_str<double>(line[start_idx + 6], quat.z());
        frame->T_self_to_enu.linear() = quat.toRotationMatrix();
        lidar_frames.push_back(frame);
    }
}

LidarFrame::Ptr LidarTrajectory::operator[](int index) const {
    return lidar_frames[index];
}

LidarFrame::Ptr LidarTrajectory::InterpolatedFrame(uint64_t timestamp) const {
    LidarFrame::Ptr this_lidar_frame = nullptr;
    LidarFrame::Ptr next_lidar_frame = nullptr;
    for (int i = 0; i + 1 < lidar_frames.size(); i++) {
        if (lidar_frames[i]->timestamp <= timestamp &&
            timestamp < lidar_frames[i + 1]->timestamp) {
            this_lidar_frame = lidar_frames[i];
            next_lidar_frame = lidar_frames[i + 1];
        }
    }

    if (this_lidar_frame == nullptr || next_lidar_frame == nullptr) {
        return nullptr;
    }

    LidarFrame::Ptr interpolated_lidar_frame = LidarFrame::New();
    interpolated_lidar_frame->FromInterpolation(
        *this_lidar_frame, *next_lidar_frame, timestamp);
    return interpolated_lidar_frame;
}

LidarFrame::Ptr LidarTrajectory::LowerBoundFrame(uint64_t timestamp) const {
    for (int i = lidar_frames.size() - 1; i >= 0; i--) {
        if (lidar_frames[i]->timestamp <= timestamp) {
            return lidar_frames[i];
        }
    }
    return nullptr;
}

LidarFrame::Ptr LidarTrajectory::UpperBoundFrame(uint64_t timestamp) const {
    for (int i = 0; i < lidar_frames.size(); i++) {
        if (lidar_frames[i]->timestamp > timestamp) {
            return lidar_frames[i];
        }
    }
    return nullptr;
}


template <typename T>
void dump_str(const std::string& str, T& output){
    std::stringstream ss(str);
    ss >> output;
}

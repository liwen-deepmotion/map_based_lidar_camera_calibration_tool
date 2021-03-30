// Author: Chi Zhang (chizhang@deepmotion.ai)

#include "system_calib.h"

#include <fstream>
#include <glog/logging.h>
#include <json/json.h>

void SystemCalib::LoadJson(const std::string& calib_file_path) {
    std::ifstream file(calib_file_path);
    LOG_IF(FATAL, !file.is_open()) << "Cannot open file: " << calib_file_path;

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string errs;
    bool success = Json::parseFromStream(builder, file, &root, &errs);
    LOG_IF(FATAL, !success) << "Cannot parse json file\n:" << errs;

    pairwise_calibrations_.FromJsonNode(root["calibration"]);

    for (const Json::Value& sensor_node : root["dm_device"]["sensors"]) {
        if (sensor_node["type"].asString() == "camera") {
            camera_calib.FromJsonNode(sensor_node);
        }
        if (sensor_node["type"].asString() == "lidar") {
            lidar_calib.FromJsonNode(sensor_node);
        }
    }

    LOG(INFO) << "System calib loaded.";
}

void SystemCalib::PairwiseCalibrations::FromJsonNode(
    const Json::Value& json_node) {
    for (const auto& sensor_pair_node : json_node) {
        std::string src_sensor_name = sensor_pair_node["sensor1"].asString();
        std::string dst_sensor_name = sensor_pair_node["sensor2"].asString();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                T.matrix()(r, c) = sensor_pair_node[
                    "T_sensor1_to_sensor2"][4 * r + c].asDouble();
            }
        }
        // Normalize the rotation component.
        T.linear() = T.rotation();
        sensor_name_pair_to_transform_map[
            std::make_pair(src_sensor_name, dst_sensor_name)] = T;
        sensor_name_pair_to_transform_map[
            std::make_pair(dst_sensor_name, src_sensor_name)] = T.inverse();
    }
}

bool SystemCalib::PairwiseCalibrations::HasTransformFromTo(
    const std::string& src_sensor_name,
    const std::string& dst_sensor_name) const {
    const auto& map = sensor_name_pair_to_transform_map;
    return map.find(std::make_pair(src_sensor_name, dst_sensor_name))
           != map.end() ||
           map.find(std::make_pair(dst_sensor_name, src_sensor_name))
           != map.end();
}

Eigen::Isometry3d SystemCalib::PairwiseCalibrations::GetTransformFromTo(
    const std::string& src_sensor_name,
    const std::string& dst_sensor_name) const {
    return sensor_name_pair_to_transform_map.at(
        std::make_pair(src_sensor_name, dst_sensor_name));
}

void SystemCalib::CameraCalib::FromJsonNode(const Json::Value& json_node) {
    name = json_node["name"].asString();
    topic = json_node["topic"].asString();
    width = json_node["width"].asInt();
    height = json_node["height"].asInt();

    double fx = json_node["intrinsics"][0].asDouble();
    double fy = json_node["intrinsics"][1].asDouble();
    double cx = json_node["intrinsics"][2].asDouble();
    double cy = json_node["intrinsics"][3].asDouble();
    K << fx, 0., cx,
         0., fy, cy,
         0., 0., 1.;
    inv_K = K.inverse();
    kappa.resize(5);
    kappa << json_node["distortions"][0].asDouble(),
             json_node["distortions"][1].asDouble(),
             json_node["distortions"][2].asDouble(),
             json_node["distortions"][3].asDouble(),
             0.;

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            T_self_to_device.matrix()(r, c) =
                json_node["sensor_to_device"][4 * r + c].asDouble();
        }
    }
    // Project the potential corrupted rotation component back to SO(3).
    T_self_to_device.linear() = T_self_to_device.rotation();
}

void SystemCalib::LidarCalib::FromJsonNode(const Json::Value& json_node) {
    name = json_node["name"].asString();
    topic = json_node["topic"].asString();
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            T_self_to_device.matrix()(r, c) =
                json_node["sensor_to_device"][4 * r + c].asDouble();
        }
    }
    // Project the potential corrupted rotation component back to SO(3).
    T_self_to_device.linear() = T_self_to_device.rotation();
}

Eigen::Isometry3d SystemCalib::LidarToCameraTransform() const {
    if (pairwise_calibrations_.HasTransformFromTo(
        lidar_calib.name, camera_calib.name)) {
        return pairwise_calibrations_.GetTransformFromTo(
            lidar_calib.name, camera_calib.name);
    } else {
        return camera_calib.T_self_to_device.inverse() *
               lidar_calib.T_self_to_device;
    }
}

Eigen::Isometry3d SystemCalib::CameraToLidarTransform() const {
    return LidarToCameraTransform().inverse();
}

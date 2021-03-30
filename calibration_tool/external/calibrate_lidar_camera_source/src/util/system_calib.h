// Author: Chi Zhang (chizhang@deepmotion.ai)

#ifndef UNIVERSAL_ODOMETRY_SYSTEM_CALIB_H
#define UNIVERSAL_ODOMETRY_SYSTEM_CALIB_H

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <json/json.h>


struct SystemCalib {
    struct PairwiseCalibrations {
        void FromJsonNode(const Json::Value& json_node);

        bool HasTransformFromTo(
            const std::string& src_sensor_name,
            const std::string& dst_sensor_name) const;

        Eigen::Isometry3d GetTransformFromTo(
            const std::string& src_sensor_name,
            const std::string& dst_sensor_name) const;

        // A map that maps at sensor name pair to their calibrated
        // transform.
        std::map<std::pair<std::string, std::string>, Eigen::Isometry3d>
            sensor_name_pair_to_transform_map;
    };

    struct CameraCalib {
        void FromJsonNode(const Json::Value& json_node);

        // ROS topic string.
        std::string topic;

        // Sensor model name.
        std::string name;

        // Image resolution.
        int width;
        int height;

        // Focal lengths and principle point position.
        Eigen::Matrix3d K;

        // Inverse of the intrinsic matrix K. In case we need to access
        // inv_K, we usually access it in a looping of camera keyframes,
        // so we cache an inv_K here to save computation from frequently
        // inversing K.
        Eigen::Matrix3d inv_K;

        // Radial distortions. 5 dimension.
        Eigen::VectorXd kappa;

        // Transform from self to the dmbox.
        Eigen::Isometry3d T_self_to_device;
    };

    struct LidarCalib {
        void FromJsonNode(const Json::Value& json_node);

        // ROS topic string.
        std::string topic;

        // Sensor model name.
        std::string name;

        // Transform from self to the dmbox.
        Eigen::Isometry3d T_self_to_device;
    };

    void LoadJson(const std::string& calib_file_path);

    Eigen::Isometry3d CameraToLidarTransform() const;

    Eigen::Isometry3d LidarToCameraTransform() const;

    PairwiseCalibrations pairwise_calibrations_;

    CameraCalib camera_calib;

    LidarCalib lidar_calib;
};

#endif  // UNIVERSAL_ODOMETRY_SYSTEM_CALIB_H

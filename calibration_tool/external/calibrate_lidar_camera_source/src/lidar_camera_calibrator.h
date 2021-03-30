// Author: Chi Zhang (chizhang@deepmotion.ai)

#ifndef SEMANTIC_SLAM_LIDAR_CAMERA_CALIBRATOR_H
#define SEMANTIC_SLAM_LIDAR_CAMERA_CALIBRATOR_H

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <ceres/ceres.h>
#include <json/json.h>

#include "util/system_calib.h"
#include "util/lidar_trajectory.h"
#include "util/line_segment_3d.h"
#include "util/line_segment_2d.h"


class LidarCameraCalibrator {
public:
    struct Transform {
        Eigen::Isometry3d ToIsometry3d() const;

        void LoadJson(const std::string& json_file_path);
        void SaveJson(const std::string& json_file_path) const;

        Eigen::Quaterniond quaternion_ldr_to_cam;
        Eigen::Vector3d translation_ldr_to_cam;
        double time_lag = 0.;
    };

    // The struct represents a 3D-2D correspondences between a 3D hd map
    // point and a 2D annotation point.
    struct PointPointCorrespondence {
        // Position of the hd map point in ENU coordinates.
        Eigen::Vector3d point_enu;

        // Normal of the hd map point in lidar coordinates. The normal
        // points to the sky if the point is from a ground element,
        // otherwise it points towards the camera (when it is from an
        // above-ground element).
        Eigen::Vector3d normal_ldr;

        // Position of the annotation point in image coordinates.
        Eigen::Vector2d point_img;

        // Ray direction the annotation point in camera coordinates.
        Eigen::Vector3d pixel_direction_cam;

        // Is the correspondence from an element on road or above road.
        bool is_on_road;

        // A weight used for optimization.
        double weight;

        // The raw id and the residual block id of this correspondence,
        // used to format the output evaluation summary, used by the
        // outer interactive annotation and inspection tool.
        int id = -1;
        ceres::ResidualBlockId residual_block_id = nullptr;
    };

    // The struct represents a 3D-2D correspondences between a 3D hd map
    // line segment and a 2D annotation line segment.
    struct LineLineCorrespondence {

        // The hd map line segment in ENU coordinats.
        LineSegment3d line_segment_enu;

        // The normal of the hd map line segment's support plane in
        // lidar coordinates. For ground elements, the support plane is
        // the ground plane passing through the line segment with its
        // normal pointing towards sky. For elements above ground, the
        // support plane is a plane passing through the line segment
        // with its normal pointing towards the camera.
        Eigen::Vector3d line_normal_ldr;

        // The annotation line segment in image coordinates.
        LineSegment2d line_segment_img;

        // Positions of the sampled points from the annotation line
        // segment. The sampled points are generated in a way sucn that
        // they are even spaced in the camera coordinates, what their 3D
        // positions are computed using an initial (rough) ground plane
        // estimate.
        std::vector<Eigen::Vector2d> sampled_points_img;

        // Pixel directions in camera coordinates that correspnod
        // one-to-one to the annotation line segment's sampled points.
        std::vector<Eigen::Vector3d> pixel_directions_cam;

        // Is the correspondence from an element on road or above road.
        bool is_on_road;

        // A weight used for optimization.
        double weight;

        // The raw id and the residual block id of this correspondence,
        // used to format the output evaluation summary, used by the
        // outer interactive annotation and inspection tool.
        int id = -1;
        ceres::ResidualBlockId residual_block_id = nullptr;
    };

    // A LidarCameraCalibrator keyframe represents a source of 3D-2D
    // correspondences between hd map elements and image annotations.
    // These correspondences are used to provide contraints to optimize
    // the relative pose between the lidar and the camera.
    struct Keyframe {
        void Load3d2dCorrespondences(const Json::Value& json_node);

        // Compute 3d-2d point-point correspondence attributes (which
        // include point normal and pixel direction) for optimization.
        void ComputePointPointCorrespondenceAttributes(
            const SystemCalib& system_calib,
            const Transform& old_transform);

        // Compute 3d-2d line-line correspondence attributes (which
        // include segment normal, sampled points and sampled points'
        // pixel directions) for optimization.
        void ComputeLineLineCorrespondenceAttributes(
            const SystemCalib& system_calib,
            const Transform& old_transform);

        // Timestamp of the image.
        uint64_t timestamp;

        // The interpolated pose of the virtual lidar frame that has the
        // same timestamp to the image, used under the context assuming
        // no time lag.
        Eigen::Isometry3d T_enu_to_ldr;

        // The poses of the keyframe closest lower and upper lidar
        // frames, used under the context where time lag is modeled.
        Eigen::Isometry3d T_enu_to_lower_ldr;
        Eigen::Isometry3d T_enu_to_upper_ldr;

        // The timestamps of the interpolated, lower and upper frames in
        // milliseconds, used to optimize time lag.
        double t;
        double t_lower;
        double t_upper;

        // The image content, used for visualization purpose.
        cv::Mat3b image;

        // 3D-2D point point correspondences.
        std::vector<PointPointCorrespondence> point_point_correspondences;

        // 3D-2D line line correspondences.
        std::vector<LineLineCorrespondence> line_line_correspondences;
    };

    Transform NewTransform() const;
    Transform OldTransform() const;

    void SetKeyframeDirectory(const std::string& keyframe_dir);
    void SetSystemCalib(SystemCalib* system_calib);
    void SetLidarTrajectory(LidarTrajectory* lidar_trajectory);
    void SetInitialTransform(const Transform& initial_transform);

    void LoadKeyframes();
    void Load3d2dCorrespondences(const std::string& correspondence_file_path);

    void Calibrate();
    void AddParameters();
    void AddReprojectionResiduals();

    void SaveEvaluationSummary(const std::string& txt_file_path);

protected:
    std::vector<uint64_t> GetKeyframeTimestamps() const;

    std::string GetKeyframeImagePath(uint64_t timestamp) const;

    bool HasTimestamp(uint64 timestamp) const;

    Keyframe& GetKeyframe(uint64 timestamp);

private:
    SystemCalib* system_calib_ = nullptr;
    LidarTrajectory* lidar_trajectory_ = nullptr;

    std::string keyframe_dir_;
    std::vector<Keyframe> keyframes_;

    Transform old_transform_;
    Transform new_transform_;

    ceres::Problem problem_;
};

#endif  // SEMANTIC_SLAM_LIDAR_CAMERA_CALIBRATOR_H

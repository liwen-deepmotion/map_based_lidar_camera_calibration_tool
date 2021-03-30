// Author: Chi Zhang (chizhang@deepmotion.ai) 7/8/20

#ifndef AUTOMATED_MAPPING_ENGINE_LLC_REPROJECTION_FACTOR_H
#define AUTOMATED_MAPPING_ENGINE_LLC_REPROJECTION_FACTOR_H

#include <Eigen/Eigen>
#include <ceres/ceres.h>

#include "correspondence_factor_base.h"
#include "lidar_camera_calibrator.h"

// Given a 3D-2D line-to-line correspondence, computes the distance
// between (1) the projection of the 3D line semgent and (2) the 2D line
// segment in image coordiantes. The segment-to-segment distance in 2D
// is decomposed into the sum of sample-point-to-segment distances.
class LlcReprojectionFactor : public CorrespondenceFactorBase {
public:
    LlcReprojectionFactor(
        LidarCameraCalibrator::LineLineCorrespondence* llc,
        LidarCameraCalibrator::Keyframe* keyframe,
        const Eigen::Matrix3d& K,
        double sample_step = 1.) {
        llc_ = llc;
        keyframe_ = keyframe;
        K_ = K;
        PrepareEnuSamplePoints(sample_step);
        PrepareSegment2d();
    }

    template<typename T>
    bool operator()(const T* q_lidar_to_cam_ptr,
                    const T* t_lidar_to_cam_ptr,
                    const T* delta_t_ptr,
                    T* residuals) const {
        const Eigen::Map<const Eigen::Quaternion<T>>
            q_lidar_to_cam(q_lidar_to_cam_ptr);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>>
            t_lidar_to_cam(t_lidar_to_cam_ptr);
        const T delta_t = *delta_t_ptr;

        // Get a time lag compensated T_enu_to_lidar.
        Eigen::Quaternion<T> q_enu_to_ldr;
        Eigen::Matrix<T, 3, 1> t_enu_to_ldr;
        TimeLagCompensatedEnuToLidarTransform<T>(
            keyframe_, delta_t, &q_enu_to_ldr, &t_enu_to_ldr);

        Eigen::Matrix<T, 3, 3> K =
            K_.template cast<T>();
        Eigen::Matrix<T, 2, 1> segment_center =
            segment_center_.template cast<T>();
        Eigen::Matrix<T, 2, 1> segment_normal =
            segment_normal_.template cast<T>();
        for (int i = 0; i < sample_points_enu_.size(); i++) {
            Eigen::Matrix<T, 3, 1> point_enu =
                sample_points_enu_[i].template cast<T>();
            Eigen::Matrix<T, 3, 1> point_ldr =
                q_enu_to_ldr * point_enu + t_enu_to_ldr;
            Eigen::Matrix<T, 3, 1> point_cam =
                q_lidar_to_cam * point_ldr + t_lidar_to_cam;
            Eigen::Matrix<T, 2, 1> point_img =
                (K * point_cam).hnormalized();
            residuals[i] = (point_img - segment_center).dot(segment_normal);
        }

        return true;
    }

    static ceres::CostFunction* CreateAutoDiffCostFunction(
        LidarCameraCalibrator::LineLineCorrespondence* llc,
        LidarCameraCalibrator::Keyframe* keyframe,
        const Eigen::Matrix3d& K,
        double sample_step) {
        LlcReprojectionFactor* factor =
            new LlcReprojectionFactor(llc, keyframe, K, sample_step);
        return new ceres::AutoDiffCostFunction<
            LlcReprojectionFactor, ceres::DYNAMIC, 4, 3, 1>(
            factor, factor->NumSamplePoints());
    }

    int NumSamplePoints() const {
        return (int)sample_points_enu_.size();
    }

    void PrepareEnuSamplePoints(double sample_step) {
        sample_points_enu_ = llc_->line_segment_enu.Sample(sample_step);
    };

    void PrepareSegment2d() {
        segment_center_ = 0.5 * (llc_->line_segment_img.head +
                                 llc_->line_segment_img.tail);
        Eigen::Vector2d segment_direction = llc_->line_segment_img.Direction();
        segment_normal_ = Eigen::Vector2d(
            segment_direction.y(), -segment_direction.x()).normalized();
    }

private:
    LidarCameraCalibrator::LineLineCorrespondence* llc_ = nullptr;
    LidarCameraCalibrator::Keyframe* keyframe_ = nullptr;

    // 3D points sampled from the 3D segment in the correspondence, in
    // ENU coordinates.
    std::vector<Eigen::Vector3d> sample_points_enu_;

    // Camera intrinsics.
    Eigen::Matrix3d K_;

    // The center-normal representation of the 2D segment.
    Eigen::Vector2d segment_center_;
    Eigen::Vector2d segment_normal_;
};

#endif //AUTOMATED_MAPPING_ENGINE_LLC_REPROJECTION_FACTOR_H

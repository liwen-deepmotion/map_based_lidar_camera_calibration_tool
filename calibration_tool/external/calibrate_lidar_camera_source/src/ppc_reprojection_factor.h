// Author: Chi Zhang (chizhang@deepmotion.ai) 7/11/20

#ifndef AUTOMATED_MAPPING_ENGINE_PPC_REPROJECTION_ERROR_H
#define AUTOMATED_MAPPING_ENGINE_PPC_REPROJECTION_ERROR_H

#include <Eigen/Eigen>
#include <ceres/ceres.h>

#include "correspondence_factor_base.h"
#include "lidar_camera_calibrator.h"

// Given a 3D-2D point-to-point correspondence, computes the distance
// between (1) the 2D projection of the 3D point, and (2) the measured
// 2D point in image coordinates.
class PpcReprojectionFactor : public CorrespondenceFactorBase {
public:
    PpcReprojectionFactor(
        LidarCameraCalibrator::PointPointCorrespondence* ppc,
        LidarCameraCalibrator::Keyframe* keyframe,
        const Eigen::Matrix3d& K) {
        ppc_ = ppc;
        keyframe_ = keyframe;
        K_ = K;
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
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals);

        // Get a time lag compensated T_enu_to_lidar.
        Eigen::Quaternion<T> q_enu_to_ldr;
        Eigen::Matrix<T, 3, 1> t_enu_to_ldr;
        TimeLagCompensatedEnuToLidarTransform<T>(
            keyframe_, delta_t, &q_enu_to_ldr, &t_enu_to_ldr);

        // Project the ENU point onto image.
        Eigen::Matrix<T, 3, 3> K = K_.template cast<T>();
        Eigen::Matrix<T, 3, 1> point_ldr =
            q_enu_to_ldr * ppc_->point_enu.template cast<T>() + t_enu_to_ldr;
        Eigen::Matrix<T, 3, 1> point_cam =
            q_lidar_to_cam * point_ldr + t_lidar_to_cam;
        Eigen::Matrix<T, 2, 1> projected_point_img =
            (K * point_cam).hnormalized();
        Eigen::Matrix<T, 2, 1> measured_point_img =
            ppc_->point_img.template cast<T>();

        residual = measured_point_img - projected_point_img;
        return true;
    }

    static ceres::CostFunction* CreateAutoDiffCostFunction(
        LidarCameraCalibrator::PointPointCorrespondence* ppc,
        LidarCameraCalibrator::Keyframe* keyframe,
        const Eigen::Matrix3d& K) {
        PpcReprojectionFactor* factor =
            new PpcReprojectionFactor(ppc, keyframe, K);
        return new ceres::AutoDiffCostFunction<
            PpcReprojectionFactor, 2, 4, 3, 1>(factor);
    }

private:
    LidarCameraCalibrator::PointPointCorrespondence* ppc_ = nullptr;
    LidarCameraCalibrator::Keyframe* keyframe_ = nullptr;

    // Camera intrinsics.
    Eigen::Matrix3d K_;
};

#endif //AUTOMATED_MAPPING_ENGINE_PPC_REPROJECTION_ERROR_H

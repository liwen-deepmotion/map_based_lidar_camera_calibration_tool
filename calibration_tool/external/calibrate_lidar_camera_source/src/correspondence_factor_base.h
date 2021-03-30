// Author: Chi Zhang (chizhang@deepmotion.ai) 7/8/20

#ifndef AUTOMATED_MAPPING_ENGINE_CORRESPONDENCE_FACTOR_BASE_H
#define AUTOMATED_MAPPING_ENGINE_CORRESPONDENCE_FACTOR_BASE_H

#include <Eigen/Eigen>
#include <ceres/ceres.h>

#include "lidar_camera_calibrator.h"


struct CorrespondenceFactorBase {
public:
    CorrespondenceFactorBase() {}

    template<typename T>
    void TimeLagCompensatedEnuToLidarTransform(
        const LidarCameraCalibrator::Keyframe* keyframe,
        const T& delta_t,
        Eigen::Quaternion<T>* q_enu_to_ldr,
        Eigen::Matrix<T, 3, 1>* t_enu_to_ldr) const {
        // Prepare lower and upper poses for pose interpolation.
        T alpha = ((T)keyframe->t + delta_t - (T)keyframe->t_lower) /
                  ((T)keyframe->t_upper - (T)keyframe->t_lower);
        Eigen::Quaternion<T> q_enu_to_lower_ldr = Eigen::Quaterniond(
            keyframe->T_enu_to_lower_ldr.linear()).template cast<T>();
        Eigen::Quaternion<T> q_enu_to_upper_ldr = Eigen::Quaterniond(
            keyframe->T_enu_to_upper_ldr.linear()).template cast<T>();
        Eigen::Matrix<T, 3, 1> t_enu_to_lower_ldr =
            keyframe->T_enu_to_lower_ldr.translation().template cast<T>();
        Eigen::Matrix<T, 3, 1> t_enu_to_upper_ldr =
            keyframe->T_enu_to_upper_ldr.translation().template cast<T>();

        // Interpolate pose.
        *q_enu_to_ldr = q_enu_to_lower_ldr.slerp(alpha, q_enu_to_upper_ldr);
        *t_enu_to_ldr = t_enu_to_lower_ldr +
                        alpha * (t_enu_to_upper_ldr - t_enu_to_lower_ldr);
    }
};

#endif //AUTOMATED_MAPPING_ENGINE_CORRESPONDENCE_FACTOR_BASE_H

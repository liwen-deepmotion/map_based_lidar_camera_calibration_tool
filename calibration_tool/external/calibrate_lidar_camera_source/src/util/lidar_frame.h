// Author: Chi Zhang (chizhang@deepmotion.ai) 11/27/18

#ifndef LIDARCAMERACALIBRATOR_LIDAR_FRAME_H
#define LIDARCAMERACALIBRATOR_LIDAR_FRAME_H

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <ceres/ceres.h>


class LidarFrame {
public:
    typedef std::shared_ptr<LidarFrame> Ptr;

    static LidarFrame::Ptr New();

    // Constructs self from the interpolation of two lidar frames. The
    // methods assumes that src.timestamp <= dst.timestamp. The method
    // outputs a FATAL error if the interpolation fails.
    void FromInterpolation(const LidarFrame& src,
                           const LidarFrame& dst,
                           uint64_t timestamp);

    // The starting timestamp of the lidar frame.
    uint64_t timestamp;

    // Transform from the lidar frame's (at the starting timestamp)
    // local coordinates to ENU.
    Eigen::Isometry3d T_self_to_enu;
};

#endif // LIDARCAMERACALIBRATOR_LIDAR_FRAME_H

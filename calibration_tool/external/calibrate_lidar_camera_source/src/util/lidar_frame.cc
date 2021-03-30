// Author: Chi Zhang (chizhang@deepmotion.ai) 11/27/18

#include "lidar_frame.h"

#include <iostream>
#include <cstdio>
#include <set>
#include <gflags/gflags.h>
#include <glog/logging.h>


LidarFrame::Ptr LidarFrame::New() {
    return std::make_shared<LidarFrame>();
}

void LidarFrame::FromInterpolation(const LidarFrame& src,
                                   const LidarFrame& dst,
                                   uint64_t timestamp) {
    if (!(src.timestamp <= timestamp && timestamp <= dst.timestamp)) {
        LOG(FATAL) << "Timestamp order corrupted. "
                      "Cannot interpolate lidar frame.";
    }
    double alpha = ((double)timestamp - src.timestamp) /
                   ((double)dst.timestamp - src.timestamp);
    this->timestamp = timestamp;
    T_self_to_enu.translation() =
        src.T_self_to_enu.translation() +
        alpha * (dst.T_self_to_enu.translation() -
                 src.T_self_to_enu.translation());
    T_self_to_enu.linear() =
        Eigen::Quaterniond(src.T_self_to_enu.rotation())
            .slerp(alpha, Eigen::Quaterniond(dst.T_self_to_enu.rotation()))
            .toRotationMatrix();
}


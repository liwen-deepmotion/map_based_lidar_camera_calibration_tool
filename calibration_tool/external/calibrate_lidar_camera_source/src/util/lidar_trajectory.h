// Author: Chi Zhang (chizhang@deepmotion.ai) 11/20/19

#ifndef POSE_GRAPH_OPTIMIZER_LIDAR_TRAJECTORY_H
#define POSE_GRAPH_OPTIMIZER_LIDAR_TRAJECTORY_H

#include <memory>
#include <vector>

#include "lidar_frame.h"

struct LidarTrajectory {
    typedef std::shared_ptr<LidarTrajectory> Ptr;

    static LidarTrajectory::Ptr New();

    // Loads the trajectory from a .csv file.
    void LoadCsv(const std::string& csv_file_path);

    // Returns the index-th lidar frame in the lidar_frames array. Note
    // that the index is not necessarily the same with the id the
    // returned lidar frame.
    LidarFrame::Ptr operator[](int index) const;

    // Returns a virtual lidar frame interpolated at `timestamp`. Return
    // a nullptr if the input timestamp is not enclosed by the lidar
    // trajectory's timestamp interval.
    LidarFrame::Ptr InterpolatedFrame(uint64_t timestamp) const;

    // Returns the closest lidar frame such that
    // `returned_frame.timestmap <= timestamp`. Returns nullptr if not
    // exist.
    LidarFrame::Ptr LowerBoundFrame(uint64_t timestamp) const;

    // Returns the closest lidar frame such that
    // `returned_frame.timestmap > timestamp`. Returns nullptr if not
    // exist.
    LidarFrame::Ptr UpperBoundFrame(uint64_t timestamp) const;

    std::vector<LidarFrame::Ptr> lidar_frames;
};

#endif  // POSE_GRAPH_OPTIMIZER_LIDAR_TRAJECTORY_H

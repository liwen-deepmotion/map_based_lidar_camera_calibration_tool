// Author: Chi Zhang (chizhang@deepmotion.ai)

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "util/system_calib.h"
#include "util/lidar_trajectory.h"
#include "lidar_camera_calibrator.h"

DEFINE_string(seq_dir, "", "");
DEFINE_string(keyframe_dir_path, "",
              "Default: ${seq_dir}/keyframes");
DEFINE_string(correspondence_json_path, "",
              "Default: ${seq_dir}/3d_2d_correspondences.json");
DEFINE_string(initial_transform_json_path, "",
              "Default: empty.");
DEFINE_string(optimized_transform_json_path, "",
              "Default: ${seq_dir}/optimized_lidar_to_camera_transform.json");
DEFINE_string(evaluation_summary_txt_path, "",
              "Default: ${seq_dir}/evaluation_summary.txt");

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::string keyframe_dir_path =
        !FLAGS_keyframe_dir_path.empty()
        ? FLAGS_keyframe_dir_path
        : FLAGS_seq_dir + "/keyframes";
    std::string correspondence_json_path =
        !FLAGS_correspondence_json_path.empty()
        ? FLAGS_correspondence_json_path
        : FLAGS_seq_dir + "/3d_2d_correspondences.json";
    std::string optimized_transform_json_path =
        !FLAGS_optimized_transform_json_path.empty()
        ? FLAGS_optimized_transform_json_path
        : FLAGS_seq_dir + "/optimized_lidar_to_camera_transform.json";
    std::string evaluation_summary_txt_path =
        !FLAGS_evaluation_summary_txt_path.empty()
        ? FLAGS_evaluation_summary_txt_path
        : FLAGS_seq_dir + "/evaluation_summary.txt";

    SystemCalib system_calib;
    std::string system_calib_path = FLAGS_seq_dir + "/sensor_calibrations.json";
    system_calib.LoadJson(system_calib_path);

    LidarTrajectory lidar_trajectory;
    std::string lidar_trajectory_file_path =
            FLAGS_seq_dir + "/lidar_trajectory.posegraph.csv";
    lidar_trajectory.LoadCsv(lidar_trajectory_file_path);

    LidarCameraCalibrator calibrator;
    calibrator.SetKeyframeDirectory(keyframe_dir_path);
    calibrator.SetSystemCalib(&system_calib);
    if (!FLAGS_initial_transform_json_path.empty()) {
        LidarCameraCalibrator::Transform initial_transform;
        initial_transform.LoadJson(FLAGS_initial_transform_json_path);
        calibrator.SetInitialTransform(initial_transform);
    }
    calibrator.SetLidarTrajectory(&lidar_trajectory);
    calibrator.LoadKeyframes();
    calibrator.Load3d2dCorrespondences(correspondence_json_path);
    calibrator.Calibrate();

    calibrator.NewTransform().SaveJson(optimized_transform_json_path);
    calibrator.SaveEvaluationSummary(evaluation_summary_txt_path);

    return 0;
}


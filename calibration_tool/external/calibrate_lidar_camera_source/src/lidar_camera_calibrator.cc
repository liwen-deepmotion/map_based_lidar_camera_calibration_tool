// Author: Chi Zhang (chizhang@deepmotion.ai)

#include "lidar_camera_calibrator.h"

#include <fstream>
#include <boost/filesystem.hpp>
#include <opencv2/highgui.hpp>

#include "llc_reprojection_factor.h"
#include "ppc_reprojection_factor.h"


Eigen::Isometry3d LidarCameraCalibrator::Transform::ToIsometry3d() const {
    Eigen::Isometry3d T_ldr_to_cam = Eigen::Isometry3d::Identity();
    T_ldr_to_cam.linear() = quaternion_ldr_to_cam.toRotationMatrix();
    T_ldr_to_cam.translation() = translation_ldr_to_cam;
    return T_ldr_to_cam;
}

void LidarCameraCalibrator::Transform::LoadJson(
    const std::string& json_file_path) {
    std::ifstream file(json_file_path);
    LOG_IF(FATAL, !file.is_open()) << "Fail to load option file: "
                                   << json_file_path;

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string errs;
    bool ok = Json::parseFromStream(builder, file, &root, &errs);
    LOG_IF(FATAL, !ok) << "Cannot parse json file:\n " << errs;

    // Populate extrinsics.
    Json::Value transform_node = root["T_lidar_to_cam"];
    CHECK_EQ(transform_node.size(), 16);
    Eigen::Isometry3d T_ldr_to_cam;
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            T_ldr_to_cam.matrix()(r, c) = transform_node[4*r + c].asDouble();
        }
    }
    quaternion_ldr_to_cam = Eigen::Quaterniond(T_ldr_to_cam.linear());
    translation_ldr_to_cam = T_ldr_to_cam.translation();
}

void LidarCameraCalibrator::Transform::SaveJson(
    const std::string& json_file_path) const {
    Eigen::Isometry3d T_lidar_to_cam = Eigen::Isometry3d::Identity();
    T_lidar_to_cam.linear() = quaternion_ldr_to_cam.toRotationMatrix();
    T_lidar_to_cam.translation() = translation_ldr_to_cam;

    Json::Value transform_node(Json::arrayValue);
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            transform_node.append(T_lidar_to_cam.matrix()(r, c));
        }
    }

    Json::Value root;
    root["T_lidar_to_cam"] = transform_node;
    std::ofstream file(json_file_path);
    LOG_IF(FATAL, !file.is_open()) << "Cannot open file for writing: "
                                   << json_file_path;
    file << root.toStyledString();
    file.close();
    LOG(INFO) << "Optimized transform has been saved to "
              << json_file_path;
}

LidarCameraCalibrator::Transform LidarCameraCalibrator::NewTransform() const {
    return new_transform_;
}

LidarCameraCalibrator::Transform LidarCameraCalibrator::OldTransform() const {
    return old_transform_;
}

void LidarCameraCalibrator::Keyframe::Load3d2dCorrespondences(
    const Json::Value& json_node) {
    for (Json::Value ppc_node : json_node["point_point_correspondences"]) {
        PointPointCorrespondence ppc;

        ppc.point_enu.x() = ppc_node["hd_map_point"][0].asDouble();
        ppc.point_enu.y() = ppc_node["hd_map_point"][1].asDouble();
        ppc.point_enu.z() = ppc_node["hd_map_point"][2].asDouble();

        ppc.point_img.x() = ppc_node["annotation_point"][0].asDouble();
        ppc.point_img.y() = ppc_node["annotation_point"][1].asDouble();

        ppc.is_on_road = ppc_node["is_on_road"].asBool();
        ppc.id = ppc_node["id"].asInt();

        point_point_correspondences.push_back(ppc);
    }
    for (Json::Value llc_node : json_node["line_line_correspondences"]) {
        LineLineCorrespondence llc;

        Json::Value head_node = llc_node["hd_map_segment"][0];
        Json::Value tail_node = llc_node["hd_map_segment"][1];
        llc.line_segment_enu.head.x() = head_node[0].asDouble();
        llc.line_segment_enu.head.y() = head_node[1].asDouble();
        llc.line_segment_enu.head.z() = head_node[2].asDouble();
        llc.line_segment_enu.tail.x() = tail_node[0].asDouble();
        llc.line_segment_enu.tail.y() = tail_node[1].asDouble();
        llc.line_segment_enu.tail.z() = tail_node[2].asDouble();

        head_node = llc_node["annotation_segment"][0];
        tail_node = llc_node["annotation_segment"][1];
        llc.line_segment_img.head.x() = head_node[0].asDouble();
        llc.line_segment_img.head.y() = head_node[1].asDouble();
        llc.line_segment_img.tail.x() = tail_node[0].asDouble();
        llc.line_segment_img.tail.y() = tail_node[1].asDouble();

        llc.is_on_road = llc_node["is_on_road"].asBool();
        llc.id = llc_node["id"].asInt();

        line_line_correspondences.push_back(llc);
    }
}

void LidarCameraCalibrator::Keyframe::ComputePointPointCorrespondenceAttributes(
    const SystemCalib& system_calib,
    const LidarCameraCalibrator::Transform& old_transform) {
    for (auto& ppc : point_point_correspondences) {
        if (ppc.is_on_road) {
            // Use a sky-point vector in ENU coordinates for the normal.
            ppc.normal_ldr = T_enu_to_ldr.linear() *
                             Eigen::Vector3d::UnitZ();
        } else {
            // Use a backwards-pointing vector in camera coordinates for
            // the normal.
            ppc.normal_ldr = old_transform.quaternion_ldr_to_cam.inverse() *
                             (-Eigen::Vector3d::UnitZ());
        }
        ppc.pixel_direction_cam = (system_calib.camera_calib.inv_K *
                                   ppc.point_img.homogeneous()).normalized();
    }
}

void LidarCameraCalibrator::Keyframe::ComputeLineLineCorrespondenceAttributes(
    const SystemCalib& system_calib,
    const LidarCameraCalibrator::Transform& old_transform) {
    for (auto& llc : line_line_correspondences) {
        // Compute normal of the line segment's support plane.
        LineSegment3d line_segment_ldr(
            T_enu_to_ldr * llc.line_segment_enu.head,
            T_enu_to_ldr * llc.line_segment_enu.tail);
        if (llc.is_on_road) {
            // Perform a double cross to make sure the computed normal
            // is pependicular to the line segment and is pointing
            // roughly up to the sky.
            Eigen::Vector3d sky_pointer_ldr =
                T_enu_to_ldr.linear() * Eigen::Vector3d::UnitZ();
            Eigen::Vector3d line_direction_ldr =
                line_segment_ldr.Direction();
            Eigen::Vector3d third_direction =
                sky_pointer_ldr.cross(line_direction_ldr);
            llc.line_normal_ldr
                = third_direction.cross(line_direction_ldr);
            if ((T_enu_to_ldr.linear().transpose() *
                 llc.line_normal_ldr).z() < 0.) {
                llc.line_normal_ldr = -llc.line_normal_ldr;
            }
        } else {
            // Perform a double cross to make sure the computed normal
            // is pependicular to the line segment and is pointing
            // roughly towards the camera.
            Eigen::Vector3d camera_pointer_ldr =
                old_transform.quaternion_ldr_to_cam.inverse() *
                (-Eigen::Vector3d::UnitZ());
            Eigen::Vector3d line_direction_ldr =
                line_segment_ldr.Direction();
            Eigen::Vector3d third_direction =
                camera_pointer_ldr.cross(line_direction_ldr);
            llc.line_normal_ldr =
                third_direction.cross(line_direction_ldr);
            if ((old_transform.quaternion_ldr_to_cam *
                 llc.line_normal_ldr).z() > 0.) {
                llc.line_normal_ldr = -llc.line_normal_ldr;
            }
        }

        // Generate sampled points and their pixel directions.
        Eigen::Matrix3d K = system_calib.camera_calib.K;
        Eigen::Matrix3d inv_K = system_calib.camera_calib.inv_K;
        Eigen::Vector3d c = old_transform.ToIsometry3d() *
                            (0.5 * (llc.line_segment_enu.head +
                                    llc.line_segment_enu.tail));
        Eigen::Vector3d n = old_transform.ToIsometry3d().linear() *
                            llc.line_normal_ldr;
        Eigen::Vector3d head_ray =
            (inv_K * llc.line_segment_img.head.homogeneous()).normalized();
        Eigen::Vector3d tail_ray =
            (inv_K * llc.line_segment_img.tail.homogeneous()).normalized();
        Eigen::Vector3d head_cam = c.dot(n) / head_ray.dot(n) * head_ray;
        Eigen::Vector3d tail_cam = c.dot(n) / tail_ray.dot(n) * tail_ray;
        double sample_step_in_meters = 1.;
        int num_sampled_points =
            1 + (int)std::ceil(llc.line_segment_enu.Length()
                               / sample_step_in_meters);
        for (int i = 0; i < num_sampled_points; i++) {
            Eigen::Vector3d p_cam =
                head_cam + (double)i / (num_sampled_points - 1) *
                           (tail_cam - head_cam);
            Eigen::Vector3d p_img = K * p_cam;
            p_img /= p_img.z();
            llc.sampled_points_img.push_back(p_img.head<2>());
            llc.pixel_directions_cam.push_back((inv_K * p_img).normalized());
        }
    }
}

void LidarCameraCalibrator::SetKeyframeDirectory(
    const std::string& keyframe_dir) {
    keyframe_dir_ = keyframe_dir;
}

void LidarCameraCalibrator::SetSystemCalib(SystemCalib* system_calib) {
    system_calib_ = system_calib;
    Eigen::Isometry3d T_lidar_to_cam = system_calib_->LidarToCameraTransform();
    old_transform_.quaternion_ldr_to_cam =
        Eigen::Quaterniond(T_lidar_to_cam.linear()).normalized();
    old_transform_.translation_ldr_to_cam =
        T_lidar_to_cam.translation();
    new_transform_.quaternion_ldr_to_cam =
        Eigen::Quaterniond(T_lidar_to_cam.linear()).normalized();
    new_transform_.translation_ldr_to_cam =
        T_lidar_to_cam.translation();
}

void LidarCameraCalibrator::SetLidarTrajectory(
    LidarTrajectory* lidar_trajectory) {
    lidar_trajectory_ = lidar_trajectory;
}

void LidarCameraCalibrator::SetInitialTransform(
    const LidarCameraCalibrator::Transform& initial_transform) {
    old_transform_ = initial_transform;
    new_transform_ = initial_transform;
}

void LidarCameraCalibrator::LoadKeyframes() {
    for (uint64_t timestamp : GetKeyframeTimestamps()) {
        Keyframe keyframe;
        keyframe.timestamp = timestamp;
        keyframe.image = cv::imread(GetKeyframeImagePath(timestamp),
                                    cv::IMREAD_COLOR);
        LidarFrame::Ptr ontime_lidar_frame =
            lidar_trajectory_->InterpolatedFrame(timestamp);
        LidarFrame::Ptr lower_lidar_frame =
            lidar_trajectory_->LowerBoundFrame(timestamp);
        LidarFrame::Ptr upper_lidar_frame =
            lidar_trajectory_->UpperBoundFrame(timestamp);
        if (ontime_lidar_frame == nullptr ||
            lower_lidar_frame == nullptr ||
            upper_lidar_frame == nullptr) {
            LOG(ERROR) << "Cannot interpolate keyframe at " << timestamp
                       << " " << ontime_lidar_frame
                       << " " << lower_lidar_frame
                       << " " << upper_lidar_frame;
            continue;
        }
        keyframe.T_enu_to_ldr =
            ontime_lidar_frame->T_self_to_enu.inverse();
        keyframe.T_enu_to_lower_ldr =
            lower_lidar_frame->T_self_to_enu.inverse();
        keyframe.T_enu_to_upper_ldr =
            upper_lidar_frame->T_self_to_enu.inverse();
        keyframe.t = ontime_lidar_frame->timestamp * 1e-6;
        keyframe.t_lower = lower_lidar_frame->timestamp * 1e-6;
        keyframe.t_upper = upper_lidar_frame->timestamp * 1e-6;
        keyframes_.push_back(keyframe);
    }
    LOG(INFO) << keyframes_.size() << " keyframes loaded.";
}

void LidarCameraCalibrator::Load3d2dCorrespondences(
    const std::string& correspondence_file_path) {
    std::ifstream file(correspondence_file_path);
    if (!file.is_open()) {
        LOG(ERROR) << "Cannot open file: " << correspondence_file_path;
        return;
    }

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string errs;
    bool ok = Json::parseFromStream(builder, file, &root, &errs);
    LOG_IF(FATAL, !ok) << "Cannot parse json file:\n " << errs;

    for (auto& keyframe_node : root["keyframes"]) {
        uint64_t timestamp = keyframe_node["timestamp"].asUInt64();
        if (!HasTimestamp(timestamp)) {
            LOG(ERROR) << "Calibrator cannot find image " << timestamp;
            continue;
        }
        Keyframe& keyframe = GetKeyframe(timestamp);
        keyframe.Load3d2dCorrespondences(keyframe_node);
        keyframe.ComputePointPointCorrespondenceAttributes(
            *system_calib_, old_transform_);
        keyframe.ComputeLineLineCorrespondenceAttributes(
            *system_calib_, old_transform_);
    }
}

std::vector<uint64_t> LidarCameraCalibrator::GetKeyframeTimestamps() const {
    std::vector<uint64_t> timestamps;
    boost::filesystem::directory_iterator iter_end;
    for (boost::filesystem::directory_iterator iter(keyframe_dir_);
         iter != iter_end; ++iter) {
        if (boost::filesystem::is_regular_file(iter->status())) {
            if (iter->path().filename().extension().string() == ".jpg") {
                timestamps.push_back(std::stoull(
                    iter->path().filename().stem().string()));
            }
        }
    }
    std::sort(timestamps.begin(), timestamps.end());
    return timestamps;
}

std::string LidarCameraCalibrator::GetKeyframeImagePath(
    uint64_t timestamp) const {
    return keyframe_dir_ + "/" + std::to_string(timestamp) + ".jpg";
}

bool LidarCameraCalibrator::HasTimestamp(uint64 timestamp) const {
    for (const auto& keyframe : keyframes_) {
        if (keyframe.timestamp == timestamp) {
            return true;
        }
    }
    return false;
}

LidarCameraCalibrator::Keyframe&
LidarCameraCalibrator::GetKeyframe(uint64 timestamp) {
    for (auto& keyframe : keyframes_) {
        if (keyframe.timestamp == timestamp) {
            return keyframe;
        }
    }
    LOG(FATAL) << "Calibrator does not have keyframe " << timestamp;
}

void LidarCameraCalibrator::Calibrate() {
    AddParameters();
    AddReprojectionResiduals();

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;
    options.max_solver_time_in_seconds = 60;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    std::cout << summary.FullReport() << std::endl;
}

void LidarCameraCalibrator::AddParameters() {
    ceres::LocalParameterization* quaternion_local_parameterization =
        new ceres::EigenQuaternionParameterization();
    problem_.AddParameterBlock(
        new_transform_.quaternion_ldr_to_cam.coeffs().data(), 4,
        quaternion_local_parameterization);
    problem_.AddParameterBlock(new_transform_.translation_ldr_to_cam.data(), 3);
    problem_.AddParameterBlock(&new_transform_.time_lag, 1);
    problem_.SetParameterBlockConstant(&new_transform_.time_lag);
}

void LidarCameraCalibrator::AddReprojectionResiduals() {
    for (auto& keyframe : keyframes_) {
        for (auto& ppc : keyframe.point_point_correspondences) {
            ppc.weight = 1.;
            ceres::CostFunction* cost_function =
                PpcReprojectionFactor::CreateAutoDiffCostFunction(
                    &ppc, &keyframe, system_calib_->camera_calib.K);
            ppc.residual_block_id = problem_.AddResidualBlock(
                cost_function, nullptr,
                new_transform_.quaternion_ldr_to_cam.coeffs().data(),
                new_transform_.translation_ldr_to_cam.data(),
                &new_transform_.time_lag);
        }
        for (auto& llc : keyframe.line_line_correspondences) {
            ceres::CostFunction* cost_function =
                LlcReprojectionFactor::CreateAutoDiffCostFunction(
                    &llc, &keyframe, system_calib_->camera_calib.K, 1.);
            llc.residual_block_id = problem_.AddResidualBlock(
                cost_function, nullptr,
                new_transform_.quaternion_ldr_to_cam.coeffs().data(),
                new_transform_.translation_ldr_to_cam.data(),
                &new_transform_.time_lag);
        }
    }
}

void LidarCameraCalibrator::SaveEvaluationSummary(
    const std::string& txt_file_path) {
    FILE* f = fopen(txt_file_path.c_str(), "w");
    CHECK_NOTNULL(f);

    double cost = 0.;
    problem_.Evaluate(ceres::Problem::EvaluateOptions(), &cost,
                      nullptr, nullptr, nullptr);
    fprintf(f, "total_cost:\n%lf\n", cost);

    fprintf(f, "[line_line_correspondence_id] cost "
               "[num_residuals] residual1 residual2 ...\n");
    for (const auto& keyframe : keyframes_) {
        for (const auto& llc : keyframe.line_line_correspondences) {
            ceres::Problem::EvaluateOptions options;
            options.residual_blocks.push_back(llc.residual_block_id);
            std::vector<double> residuals;
            problem_.Evaluate(options, &cost, &residuals, nullptr, nullptr);
            fprintf(f, "[%3d] %8.4lf [%2d] ", llc.id, cost, residuals.size());
            for (double residual : residuals) {
                fprintf(f, "%8.4lf ", residual);
            }
            fprintf(f, "\n");
        }
    }

    fprintf(f, "[point_point_correspondence_id] cost "
               "[num_residuals] residual1 residual2 ...\n");
    for (const auto& keyframe : keyframes_) {
        for (const auto& ppc : keyframe.point_point_correspondences) {
            ceres::Problem problem_;
            ceres::Problem::EvaluateOptions options;
            options.residual_blocks.push_back(ppc.residual_block_id);
            std::vector<double> residuals;
            problem_.Evaluate(options, &cost, &residuals, nullptr, nullptr);
            fprintf(f, "[%3d] %8.4lf [%2d] ", ppc.id, cost, residuals.size());
            for (double residual : residuals) {
                fprintf(f, "%8.4lf ", residual);
            }
            fprintf(f, "\n");
        }
    }

    fclose(f);
}

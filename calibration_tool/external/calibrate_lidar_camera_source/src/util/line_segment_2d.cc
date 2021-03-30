// Author: Chi Zhang (chizhang@deepmotion.ai) 6/23/18

#include "line_segment_2d.h"

#include <cfloat>
#include <glog/logging.h>

LineSegment2d::LineSegment2d() : head(Eigen::Vector2d::Zero()),
                                 tail(Eigen::Vector2d::Zero()) {
}

LineSegment2d::LineSegment2d(const Eigen::Vector2d& head,
                             const Eigen::Vector2d& tail)
    : head(head), tail(tail) {
}

bool LineSegment2d::Valid() const {
    return Length() > 1e-5;
}

double LineSegment2d::Length() const {
    return (tail - head).norm();
}

Eigen::Vector2d LineSegment2d::Direction() const {
    return (tail - head).normalized();
}

double LineSegment2d::Distance(const Eigen::Vector2d& point) const {
    Eigen::Vector2d line_direction = Direction();
    double lambda = (point - head).dot(line_direction);
    if (0. <= lambda && lambda <= Length()) {
        Eigen::Vector2d foot_point = head + lambda * line_direction;
        return (point - foot_point).norm();
    } else {
        return std::min((point - head).norm(),
                        (point - tail).norm());
    }
}

std::vector<Eigen::Vector2d> LineSegment2d::Sample(
    int num_points_to_sample) const {
    CHECK(num_points_to_sample > 1);
    Eigen::Vector2d sample_step = (tail - head) / (num_points_to_sample - 1);
    std::vector<Eigen::Vector2d> sampled_points;
    for (int i = 0; i < num_points_to_sample; i++) {
        sampled_points.push_back(head + i * sample_step);
    }
    return sampled_points;
}


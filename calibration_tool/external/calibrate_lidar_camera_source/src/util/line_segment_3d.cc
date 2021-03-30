// Author: Chi Zhang (chizhang@deepmotion.ai) 5/11/18

#include "line_segment_3d.h"


LineSegment3d::LineSegment3d() {
    head = Eigen::Vector3d::Zero();
    tail = Eigen::Vector3d::Zero();
}

LineSegment3d::LineSegment3d(const Eigen::Vector3d& head,
                             const Eigen::Vector3d& tail)
    : head(head), tail(tail) {
}

double LineSegment3d::Length() const {
    return (tail - head).norm();
}

Eigen::Vector3d LineSegment3d::Direction() const {
    return (tail - head).normalized();
}

double LineSegment3d::Distance(const Eigen::Vector3d& point) const {
    Eigen::Vector3d line_direction = Direction();
    double lambda = (point - head).dot(line_direction);
    if (0. <= lambda && lambda <= Length()) {
        Eigen::Vector3d foot_point = head + lambda * line_direction;
        return (point - foot_point).norm();
    } else {
        return std::min((point - head).norm(),
                        (point - tail).norm());
    }
}

void LineSegment3d::Reverse() {
    head.swap(tail);
}

std::vector<Eigen::Vector3d>
LineSegment3d::Sample(double preferred_sample_step) const {
    int num_sample_points = 1 + (int)(Length() / preferred_sample_step);
    num_sample_points = std::max(2, num_sample_points);
    Eigen::Vector3d sample_step = (tail - head) / (num_sample_points - 1);

    std::vector<Eigen::Vector3d> sample_points;
    sample_points.push_back(head);
    for (int i = 1; i < num_sample_points; i++) {
        sample_points.push_back(head + i * sample_step);
    }
    return sample_points;
}

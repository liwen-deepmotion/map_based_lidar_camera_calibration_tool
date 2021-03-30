// Author: Chi Zhang (chizhang@deepmotion.ai) 5/11/18

#ifndef SEMANTICSLAM_LINE_SEGMENT_3D_H
#define SEMANTICSLAM_LINE_SEGMENT_3D_H

#include <Eigen/Eigen>


struct LineSegment3d {
public:
    LineSegment3d();

    LineSegment3d(const Eigen::Vector3d& head, const Eigen::Vector3d& tail);

    double Length() const;

    // Returns the normalized direction vector from head to tail.
    Eigen::Vector3d Direction() const;

    // Returns the distance between the query point and its closest
    // point on the line segment.
    double Distance(const Eigen::Vector3d& point) const;

    // Swaps head and tail.
    void Reverse();

    // Samples the segment uniformly with a step as close to the input
    // `preferred_sample_step` as possible. Head and tail vertices will
    // always be included in the sample result.
    std::vector<Eigen::Vector3d> Sample(double preferred_sample_step) const;

    Eigen::Vector3d head;
    Eigen::Vector3d tail;
};

#endif // SEMANTICSLAM_LINE_SEGMENT_3D_H

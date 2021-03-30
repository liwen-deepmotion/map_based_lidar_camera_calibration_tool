// Author: Chi Zhang (chizhang@deepmotion.ai) 6/23/18

#ifndef SEMANTICSLAM_LINE_SEGMENT_2D_H
#define SEMANTICSLAM_LINE_SEGMENT_2D_H

#include <Eigen/Eigen>

class LineSegment2d {
public:
    LineSegment2d();

    LineSegment2d(const Eigen::Vector2d& head, const Eigen::Vector2d& tail);

    bool Valid() const;

    double Length() const;

    Eigen::Vector2d Direction() const;

    // Returns the distance between the query point and its closest
    // point on the line segment.
    double Distance(const Eigen::Vector2d& point) const;

    std::vector<Eigen::Vector2d> Sample(int num_points_to_sample) const;

    Eigen::Vector2d head;
    Eigen::Vector2d tail;
};

#endif // SEMANTICSLAM_LINE_SEGMENT_2D_H

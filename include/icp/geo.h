/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace icp {
    template<const int Dim>
    using Vector = Eigen::Vector<double, Dim>;
    using Vector2 = Vector<2>;
    using Vector3 = Vector<3>;

    template<const int Dim>
    using RBTransform = Eigen::Transform<double, Dim, Eigen::Isometry>;
    using RBTransform2 = RBTransform<2>;
    using RBTransform3 = RBTransform<3>;

    template<const int Dim>
    using PointCloud = Eigen::Matrix<double, Dim, Eigen::Dynamic>;
    using PointCloud2 = PointCloud<2>;
    using PointCloud3 = PointCloud<3>;

    // TODO: eliminate?
    template<const int Dim>
    Vector<Dim> get_centroid(const PointCloud<Dim>& points) {
        return points.rowwise().mean();
    }
}

/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "icp/dim.h"

namespace icp {
    template<const Dimension Dim>
    using Vector = Eigen::Vector<double, Dim>;
    using Vector2 = Vector<Dimension::TwoD>;
    using Vector3 = Vector<Dimension::ThreeD>;

    template<const Dimension Dim>
    using RBTransform = Eigen::Transform<double, Dim, Eigen::Isometry>;
    using RBTransform2 = RBTransform<Dimension::TwoD>;
    using RBTransform3 = RBTransform<Dimension::ThreeD>;

    template<const Dimension Dim>
    using PointCloud = Eigen::Matrix<double, Dim, Eigen::Dynamic>;
    using PointCloud2 = PointCloud<Dimension::TwoD>;
    using PointCloud3 = PointCloud<Dimension::ThreeD>;

    // If we use `auto` like this we get much better template argument deduction.
    // Allows arbitrary dimension but that's fine.
    template<const auto Dim>
    Eigen::Vector<double, Dim> get_centroid(
        const Eigen::Matrix<double, Dim, Eigen::Dynamic>& points) {
        return points.rowwise().mean();
    }
}

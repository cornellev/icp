/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Eigen/src/Core/util/Constants.h"

namespace icp {
    template<const size_t Dim>
    using Vector = Eigen::Vector<double, Dim>;

    template<const size_t Dim>
    using RBTransform = Eigen::Transform<double, Dim, Eigen::Isometry>;

    template<const size_t Dim>
    using PointCloud = Eigen::Matrix<double, Dim, Eigen::Dynamic>;

    template<const size_t Dim>
    Vector<Dim> get_centroid(const PointCloud<Dim>& points) {
        return points.colwise().mean();
    }
}

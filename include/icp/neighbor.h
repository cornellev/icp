/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include <Eigen/Core>
#include <vector>

namespace icp {
    struct Neighbors {
        std::vector<double> distances;
        std::vector<Eigen::Index> indices;
    };
}

/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include <vector>

namespace icp {
    struct Neighbors {
        std::vector<float> distances;
        std::vector<size_t> indices;
    };
}

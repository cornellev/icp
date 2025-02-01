/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include "icp/geo.h"

namespace icp {
    Vector get_centroid(const std::vector<Vector>& points) {
        auto dim = points.empty() ? 0 : points[0].size();
        Vector sum = Vector::Zero(dim);
        for (const Vector& point: points) {
            sum += point;
        }
        return sum / points.size();
    }
}

/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <numeric>
#include "icp/geo.h"

namespace icp {
    Vector get_centroid(const std::vector<Vector>& points) {
        Vector sum = Vector::Zero();
        for (const Vector& point: points) {
            sum += point;
        }
        return sum / points.size();
    }
}

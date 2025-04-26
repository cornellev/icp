/**
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "icp/icp.h"
#include "algo/kdtree.h"
#include "icp/config.h"

namespace icp {
    class Vanilla final : public ICP2 {
    public:
        Vanilla();
        Vanilla(const Config& config);
        ~Vanilla();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();
        void compute_match_for_point(size_t i);

        std::unique_ptr<KdTree<Vector>> target_kdtree_;

        PointCloud a_current;
    };
}
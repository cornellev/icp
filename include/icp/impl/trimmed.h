/**
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "icp/icp.h"
#include "icp/config.h"

namespace icp {
    class Trimmed final : public ICP2 {
    public:
        Trimmed(double overlap_rate);
        Trimmed(const Config& config);
        ~Trimmed();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();

        double overlap_rate;
        PointCloud a_current;
    };
}

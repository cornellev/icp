/**
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include "icp/icp.h"

namespace icp {
    class Vanilla final : public ICP {
    public:
        Vanilla();
        Vanilla(const Config& config);
        ~Vanilla();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();

        std::vector<icp::Vector> a_current;
        // icp::Vector a_current_cm;
        // icp::Vector corr_cm = icp::Vector::Zero();
        // size_t n;
    };
}
/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <memory>
#include <Eigen/Dense>
#include "icp/icp.h"
#include "icp/config.h"
#include "algo/kdtree.h"

namespace icp {

    class Vanilla3d final : public ICP3 {
    public:
        Vanilla3d(const Config& config);
        Vanilla3d();
        ~Vanilla3d() override;

    protected:
        void setup() override;
        void iterate() override;

    private:
        PointCloud c;
        std::unique_ptr<KdTree<Vector>> target_kdtree_;
        double current_cost_;
        std::unique_ptr<icp::KdTree<Eigen::Vector3d>> kdtree_;

        Neighbors nearest_neighbor(const PointCloud& src);
        static double dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);
        static RBTransform best_fit_transform(const PointCloud& A, const PointCloud& B);
        void calculate_cost(const std::vector<double>& distances);
    };

}  // namespace icp
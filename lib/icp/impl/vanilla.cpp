/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */
#include <iostream>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "icp/geo.h"
#include "icp/impl/vanilla.h"

namespace icp {
    Vanilla::Vanilla([[maybe_unused]] const Config& config): ICP() {}
    Vanilla::Vanilla(): ICP() {}
    Vanilla::~Vanilla() {}

    void Vanilla::setup() {
        a_current = transform * a;
        matches.resize(a.cols());

        compute_matches();
    }

    void Vanilla::iterate() {
        if (a.cols() == 0 || b.cols() == 0) {
            return;
        }

        a_current = transform * a;
        Vector a_current_cm = get_centroid(a_current);

        compute_matches();

        Vector matched_b_cm = Vector::Zero();
        for (size_t i = 0; i < matches.size(); i++) {
            matched_b_cm += b.col(matches[i].pair);
        }
        matched_b_cm /= matches.size();

        Eigen::Matrix2d N = Eigen::Matrix2d::Zero();
        for (ptrdiff_t i = 0; i < a.cols(); i++) {
            N += (a_current.col(i) - a_current_cm)
                 * (b.col(matches[i].pair) - matched_b_cm).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2d> svd = N.jacobiSvd(Eigen::ComputeFullU
                                                            | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        Eigen::Matrix2d R = V * U.transpose();

        if (R.determinant() < 0) {
            V.col(1) *= -1;
            R = V * U.transpose();
        }

        RBTransform step;
        step.linear() = R;
        step.translation() = matched_b_cm - R * a_current_cm;

        transform = step * transform;
    }

    void Vanilla::compute_matches() {
        if (a.cols() == 0 || b.cols() == 0) {
            return;
        }
        std::vector<Vector> b_vec(b.cols());
        for (ptrdiff_t i = 0; i < b.cols(); i++) {
            b_vec[i] = b.col(i);
        }
        icp::KdTree<Eigen::Vector2d> kdtree(b_vec, 2);

        for (Eigen::Index i = 0; i < a.cols(); ++i) {
            const Eigen::Vector2d query = a.col(i);
            double min_dist = 0.0;
            int idx = kdtree.search(query, &min_dist);

            matches[i].cost = std::sqrt(min_dist);
            matches[i].pair = idx;
        }
    }
}
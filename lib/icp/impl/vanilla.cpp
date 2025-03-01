/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */
#include <iostream>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <vector>
#include "icp/geo.h"

#include "icp/impl/vanilla.h"

/* #name Vanilla */
/* #register vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
    Vanilla::Vanilla([[maybe_unused]] const Config& config): ICP() {}
    Vanilla::Vanilla(): ICP() {}
    Vanilla::~Vanilla() {}

    void Vanilla::setup() {
        a_current.resize(a.size());

        for (size_t i = 0; i < a.size(); i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        matches.resize(a.size());
        compute_matches();
    }

    void Vanilla::iterate() {
        const size_t n = a.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        auto a_current_cm = get_centroid(a_current);

        compute_matches();

        icp::Vector corr_cm = icp::Vector::Zero(2);
        for (size_t i = 0; i < matches.size(); i++) {
            corr_cm += b[matches[i].pair];
        }
        corr_cm /= matches.size();

        Matrix N = Matrix::Zero(2, 2);
        for (size_t i = 0; i < n; i++) {
            N += (a_current[i] - a_current_cm) * (b[matches[i].pair] - corr_cm).transpose();
        }
        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();

        if (R.determinant() < 0) {
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            R = V * U.transpose();
        }

        RBTransform step(corr_cm - R * a_current_cm, R);
        transform = transform.and_then(step);
    }

    void Vanilla::compute_matches() {
        const size_t n = a.size();
        const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            size_t best_j = (matches.empty() || matches[i].pair >= m) ? 0 : matches[i].pair;
            double best_dist = (b[best_j] - a_current[i]).squaredNorm();

            for (size_t j = 0; j < m; j++) {
                double dist_ij = (b[j] - a_current[i]).squaredNorm();
                if (dist_ij < best_dist) {
                    best_dist = dist_ij;
                    best_j = j;
                }
            }
            matches[i].cost = best_dist;
            matches[i].pair = best_j;
        }
    }
}
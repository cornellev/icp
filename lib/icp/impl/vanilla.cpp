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
        a_next.resize(a.size());

        for (size_t i = 0; i < a.size(); i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        matches.resize(a.size());
        // compute_matches();
        compute_matches(a_current, matches);
    }

    void Vanilla::iterate() {
        const size_t n = a.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        auto a_current_cm = get_centroid(a_current);

        // compute_matches();
        compute_matches(a_current, matches);

        const RBTransform predicted_step = last_step;
        for (size_t i = 0; i < a.size(); i++) {
            a_next[i] = predicted_step.apply_to(a_current[i]);  // 直接应用最近变换预测
        }

        icp::Vector corr_cm = icp::Vector::Zero(2);
        for (size_t i = 0; i < matches.size(); i++) {
            corr_cm += b[matches[i].pair];
        }
        corr_cm /= matches.size();

        Matrix N = Matrix::Zero(2, 2);
        for (size_t i = 0; i < n; i++) {
            N += (a_current[i] - a_current_cm) * (b[matches[i].pair] - corr_cm).transpose();
        }

        if (N.isZero(1e-6)) {
            std::cerr << "Warning: Zero covariance matrix\n";
        }

        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        if (!svd.computeU() || !svd.computeV()) {
            std::cerr << "SVD computation failed\n";
            return;
        }
        const Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();

        if (R.determinant() < 0) {
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            R = V * U.transpose();
        }

        // RBTransform step(corr_cm - R * a_current_cm, R);
        // transform = transform.and_then(step);

        RBTransform new_step(corr_cm - R * a_current_cm, R);
        last_step = new_step;                      // 保存最近变换
        transform = transform.and_then(new_step);  // 更新累计变换
    }

    // void Vanilla::compute_matches() {
    //     const size_t n = a.size();
    //     const size_t m = b.size();

    //     for (size_t i = 0; i < n; i++) {
    //         matches[i].point = i;
    //         matches[i].cost = std::numeric_limits<double>::infinity();
    //         size_t best_j = (matches.empty() || matches[i].pair >= m) ? 0 : matches[i].pair;
    //         double best_dist = (b[best_j] - a_current[i]).squaredNorm();

    //         for (size_t j = 0; j < m; j++) {
    //             double dist_ij = (b[j] - a_current[i]).squaredNorm();
    //             if (dist_ij < best_dist) {
    //                 best_dist = dist_ij;
    //                 best_j = j;
    //             }
    //         }
    //         matches[i].cost = best_dist;
    //         matches[i].pair = best_j;
    //     }
    // }
    void Vanilla::compute_matches(const std::vector<Vector>& source, std::vector<Match>& results) {
        const size_t n = source.size();
        results.resize(n);

#pragma omp parallel for
        for (size_t i = 0; i < n; ++i) {
            assert(i < source.size() && "Source index out of bounds");
            size_t best_j = 0;
            double best_dist = std::numeric_limits<double>::max();

            if (!results.empty() && results[i].pair < b.size()) {
                best_j = results[i].pair;
                best_dist = (b[best_j] - source[i]).squaredNorm();
            }

            for (size_t j = 0; j < b.size(); ++j) {
                assert(j < b.size() && "Target index out of bounds");
                const double dist = (b[j] - source[i]).squaredNorm();
                if (dist < best_dist) {
                    best_dist = dist;
                    best_j = j;
                }
            }

            results[i] = {i, best_j, best_dist};
        }
    }
}
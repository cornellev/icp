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
#include <memory>
#include "icp/geo.h"

#include "icp/impl/vanilla.h"

namespace icp {
    Vanilla::Vanilla([[maybe_unused]] const Config& config): ICP() {}
    Vanilla::Vanilla(): ICP() {}
    Vanilla::~Vanilla() {}

    void Vanilla::rebuild_kdtree() {
        // TODO: kdtree should take point cloud
        std::vector<Vector> b_vec(b.cols());
        for (size_t i = 0; i < b.cols(); i++) {
            b_vec[i] = b.col(i);
        }
        target_kdtree_ = std::make_unique<KdTree<Vector>>(b_vec, 4);
    }

    void Vanilla::setup() {
        a_current = transform * a;
        matches.resize(a.size());

        compute_matches();
    }

    void Vanilla::iterate() {
        const size_t n = a.size();

        if (n == 0 || b.empty()) {
            return;
        }

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        auto a_current_cm = get_centroid(a_current);

        compute_matches();

        icp::Vector corr_cm = icp::Vector::Zero(2);
        for (size_t i = 0; i < matches.size(); i++) {
            if (matches[i].pair < b.size()) {
                corr_cm += b[matches[i].pair];
            }
        }
        corr_cm /= matches.size();

        Matrix N = Matrix::Zero(2, 2);
        for (size_t i = 0; i < n; i++) {
            if (matches[i].pair < b.size()) {
                N += (a_current[i] - a_current_cm) * (b[matches[i].pair] - corr_cm).transpose();
            }
        }

        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();

        if (R.determinant() < 0) {
            V.col(V.cols() - 1) *= -1;
            R = V * U.transpose();
        }

        Vector trans = corr_cm - R * a_current_cm;
        RBTransform step(trans, R);
        transform = transform.and_then(step);
    }

    void Vanilla::compute_matches() {
        const size_t n = a.size();
        const size_t m = b.size();

        if (n == 0 || m == 0) {
            return;
        }

        if (matches.size() != n) {
            matches.resize(n);
        }

        if (target_kdtree_) {
            bool kdtree_failed = false;

            for (size_t i = 0; i < n; ++i) {
                try {
                    float min_dist = 0;
                    size_t best_j = target_kdtree_->find_nearest(a_current[i], &min_dist);

                    if (best_j < m) {
                        matches[i] = {i, best_j, static_cast<double>(min_dist)};
                    } else {
                        kdtree_failed = true;
                        break;
                    }
                } catch (...) {
                    kdtree_failed = true;
                    break;
                }
            }

            if (!kdtree_failed) {
                return;
            }

            std::cerr << "KdTree search failed, falling back to linear search" << std::endl;
            target_kdtree_ = nullptr;
        }

        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();

            for (size_t j = 0; j < m; j++) {
                double dist_ij = (b[j] - a_current[i]).squaredNorm();

                if (dist_ij < matches[i].cost) {
                    matches[i].cost = dist_ij;
                    matches[i].pair = j;
                }
            }
        }
    }
}
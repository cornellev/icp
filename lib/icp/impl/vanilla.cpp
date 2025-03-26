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
        for (ptrdiff_t i = 0; i < b.cols(); i++) {
            b_vec[i] = b.col(i);
        }
        target_kdtree_ = std::make_unique<KdTree<Vector>>(b_vec, 4);
    }

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
        Vector a_current_cm = get_centroid<2>(a_current);

        compute_matches();

        Vector matched_b_cm = Vector::Zero();
        for (size_t i = 0; i < matches.size(); i++) {
            matched_b_cm += b.col(matches[i].pair);
        }
        matched_b_cm /= matches.size();

        Eigen::Matrix2d N = (a_current.colwise() - a_current_cm)
                            * (b.colwise() - matched_b_cm).transpose();

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

        // TODO: why :skull:
        if (target_kdtree_) {
            bool kdtree_failed = false;

            for (ptrdiff_t i = 0; i < a.cols(); i++) {
                matches[i].point = i;

                try {
                    float min_dist = 0;
                    ptrdiff_t best_j = target_kdtree_->find_nearest(a_current.col(i), &min_dist);

                    if (best_j < b.cols()) {
                        matches[i].pair = best_j;
                        matches[i].cost = min_dist;
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

        for (ptrdiff_t i = 0; i < a.cols(); i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();

            for (ptrdiff_t j = 0; j < b.cols(); j++) {
                double dist_ij = (b.col(j) - a_current.col(i)).squaredNorm();

                if (dist_ij < matches[i].cost) {
                    matches[i].cost = dist_ij;
                    matches[i].pair = j;
                }
            }
        }
    }
}
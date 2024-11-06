/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "icp/impl/trimmed.h"

// TODO: should these docs be moved to the headers?

/* #name Trimmed */

/* #desc Trimmed ICP is identical to \ref vanilla_icp with the addition of an
overlap rate parameter, which specifies the percentage of points between the two
point sets that have correspondences. When the overlap rate is 1, the algorithm
reduces to vanilla. */

namespace icp {

    Trimmed::Trimmed(double overlap_rate): ICP(), overlap_rate(overlap_rate) {}
    Trimmed::Trimmed(const Config& config)
        : ICP(), overlap_rate(config.get<double>("overlap_rate", 0.7)) {}
    Trimmed::~Trimmed() {}

    void Trimmed::setup() {
        if (a_current.size() < a.size()) {
            a_current.resize(a.size());
        }

        b_cm = get_centroid(b);
    }

    void Trimmed::iterate() {
        const size_t n = a.size();
        const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        /* #step Matching Step: see \ref vanilla_icp for details. */
        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].sq_dist = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                // Point-to-point matching
                double dist_ij = (b[j] - a_current[i]).squaredNorm();

                if (dist_ij < matches[i].sq_dist) {
                    matches[i].sq_dist = dist_ij;
                    matches[i].pair = j;
                }
            }
        }

        /*
            #step
            Trimming Step

            Matches are considered in increasing order of distance.

            Sources:
            https://ieeexplore.ieee.org/abstract/document/1047997
        */
        std::sort(matches.begin(), matches.end(),
            [](const auto& a, const auto& b) { return a.sq_dist < b.sq_dist; });
        size_t new_n = static_cast<size_t>(overlap_rate * n);
        new_n = std::max<size_t>(new_n, 1);

        // yeah, i know this is inefficient. we'll get back to it later.
        std::vector<icp::Vector> trimmed_current(new_n);
        std::vector<icp::Vector> trimmed_b(new_n);
        for (size_t i = 0; i < new_n; i++) {
            trimmed_current[i] = a_current[matches[i].point];
            trimmed_b[i] = b[matches[i].point];
        }

        icp::Vector trimmed_cm = get_centroid(trimmed_current);
        icp::Vector trimmed_b_cm = get_centroid(trimmed_b);

        /* #step SVD: see \ref vanilla_icp for details. */
        Matrix N{};
        for (size_t i = 0; i < new_n; i++) {
            N += (trimmed_current[i] - trimmed_cm) * (trimmed_b[i] - trimmed_b_cm).transpose();
        }

        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();

        /* #step Reflection Handling: see \ref vanilla_icp for details. */
        if (R.determinant() < 0) {
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            R = V * U.transpose();
        }

        transform.rotation = R * transform.rotation;

        /* #step Transformation Step: see \ref vanilla_icp for details. */
        transform.translation += trimmed_b_cm - R * trimmed_cm;
    }
}
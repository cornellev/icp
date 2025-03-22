/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include "icp/geo.h"

#include "icp/impl/trimmed.h"

/* #name Trimmed */
/* #register trimmed */

/* #desc Trimmed ICP is identical to \ref vanilla_icp with the addition of an
overlap rate parameter, which specifies the percentage of points between the two
point sets that have correspondences. When the overlap rate is `1`, the algorithm
reduces to vanilla. */

namespace icp {

    /* #conf "overlap_rate" A `double` between `0.0` and `1.0` for
     * the overlap rate. The default is `1.0`. */
    Trimmed::Trimmed(const Config& config)
        : ICP(), overlap_rate(config.get<double>("overlap_rate", 0.9)) {}

    Trimmed::~Trimmed() {}

    void Trimmed::setup() {
        a_current = transform * a;

        compute_matches();
    }

    void Trimmed::iterate() {
        const size_t n = a.cols();

        a_current = transform * a;

        /* #step Matching Step: see \ref vanilla_icp for details. */
        compute_matches();

        /*
            #step
            Trimming Step

            Matches are considered in increasing order of distance.

            Sources:
            https://ieeexplore.ieee.org/abstract/document/1047997
        */
        std::sort(matches.begin(), matches.end(),
            [](const auto& a, const auto& b) { return a.cost < b.cost; });
        size_t new_n = static_cast<size_t>(overlap_rate * n);
        new_n = std::max<size_t>(new_n, 1);  // TODO: bad for scans with 0 points

        // yeah, i know this is inefficient. we'll get back to it later.
        PointCloud trimmed_a_current(new_n);
        PointCloud trimmed_b(new_n);
        for (size_t i = 0; i < new_n; i++) {
            trimmed_a_current.col(i) = a_current.col(matches[i].point);
            trimmed_b.col(i) = b.col(matches[i].pair);
        }

        Vector trimmed_a_cm = get_centroid<2>(trimmed_a_current);
        Vector trimmed_b_cm = get_centroid<2>(trimmed_b);

        /* #step SVD: see \ref vanilla_icp for details. */
        Eigen::Matrix2d N = (trimmed_a_current.colwise() - trimmed_a_cm)
                            * (trimmed_b.colwise() - trimmed_b_cm).transpose();

        Eigen::JacobiSVD<Eigen::Matrix2d> svd = N.jacobiSvd(Eigen::ComputeFullU
                                                            | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        Eigen::Matrix2d R = V * U.transpose();

        /* #step Reflection Handling: see \ref vanilla_icp for details. */
        if (R.determinant() < 0) {
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            R = V * U.transpose();
        }

        /* #step Transformation Step: see \ref vanilla_icp for details. */
        RBTransform step;
        step.linear() = R;
        step.translation() = trimmed_b_cm - R * trimmed_a_cm;

        transform = step * transform;
    }

    void Trimmed::compute_matches() {
        const size_t n = a.size();
        const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                // Point-to-point matching
                double dist_ij = (b.col(j) - a_current.col(i)).squaredNorm();

                if (dist_ij < matches[i].cost) {
                    matches[i].cost = dist_ij;
                    matches[i].pair = j;
                }
            }
        }
    }
}

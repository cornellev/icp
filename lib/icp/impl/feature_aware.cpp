/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include "icp/geo.h"

/* #name Feature-Aware */
/* #register feature_aware */

/* #desc Builds on top of \ref trimmed_icp. In addition to matching points based on a point-to-point
distance criteria, matches them based on a local "feature vector."
*/

#include "icp/impl/feature_aware.h"

namespace icp {
    constexpr double DEFAULT_OVERLAP_RATE = 0.9;
    constexpr double DEFAULT_FEATURE_WEIGHT = 0.7;
    constexpr int DEFAULT_SYMMETRIC_NEIGHBORS = 10;
    constexpr double MIN_NORM_THRESHOLD = 1e-6;

    FeatureAware::FeatureAware(double overlap_rate, double feature_weight, int symmetric_neighbors)
        : overlap_rate(overlap_rate),
          symmetric_neighbors(symmetric_neighbors),
          feature_weight(feature_weight),
          neighbor_weight(1 - feature_weight) {}

    /* #conf "overlap_rate" A `double` between `0.0` and `1.0` for the overlap rate. The default is
     * `1.0`. */
    /* #conf "feature_weight" A `double` between `0.0` and `1.0` with default value `0.7`. Decides
     * how much to weight feature cost versus point-to-point cost. */
    /* #conf "symmetric_neighbors" An `int` with default value `10`. Decides how many neighbors to
     * use on each side of a point when constructing the feature vector. */
    FeatureAware::FeatureAware(const Config& config)
        : FeatureAware(config.get<double>("overlap_rate", DEFAULT_OVERLAP_RATE),
              config.get<double>("feature_weight", DEFAULT_FEATURE_WEIGHT),
              config.get<int>("symmetric_neighbors", DEFAULT_SYMMETRIC_NEIGHBORS)) {}

    FeatureAware::~FeatureAware() = default;

    void FeatureAware::setup() {
        a_current = transform * a;
        matches.resize(a.cols());

        // TODO: is relying on NRVO a good idea?
        a_features = compute_features(a_current);
        b_features = compute_features(b);

        normalized_feature_dists = compute_norm_dists<Eigen::Dynamic>(a_features, b_features);
        double max = normalized_feature_dists.maxCoeff();
        if (max > MIN_NORM_THRESHOLD) {
            normalized_feature_dists /= max;
        }

        // TODO: should we rely on NRVO here?
        compute_matches();
    }

    void FeatureAware::iterate() {
        a_current = transform * a;

        /*
            #step
            Matching Step:

            Matches are computed based on a weighted average of the point-to-point cost metric and
           the feature cost metric (with weight `feature_weight` given to the feature vector
           cost metric).

            The feature vector for each point is computed as follows. Let \f$ p_i \f$ be the i-th
           point in the scan, ordered by angle from the scan origin (LiDAR center), and let \f$ c
           \f$ be the centroid of the scan. Then, we can define \f$ q_i = p_i - c \f$. Also, let \f$
           n \f$ be the number of `symmetric_neighbors`. The feature vector for \f$ p_k \f$ is then
           \f$ f_k = \begin{bmatrix} |q_{k - n}| - |q_k| & \ldots & |q_{k - 1}| - |q_k| & |q_{k+1}|
           - |q_k| & \ldots & |q_{k + n}| - |q_k| \end{bmatrix} \f$. The feature cost metric between
           two feature vectors \f$ f_a \f$ and \f$ f_b \f$ is simply \f$ |f_a - f_b| \f$.
         */
        compute_matches();

        /*
            #step
            Trimming Step: see \ref trimmed_icp for details.
        */
        std::sort(matches.begin(), matches.end(),
            [](const auto& a, const auto& b) { return a.cost < b.cost; });
        auto new_n = static_cast<Eigen::Index>(overlap_rate * static_cast<double>(a.cols()));
        new_n = std::max<Eigen::Index>(new_n, 1);  // TODO: bad for scans with 0 points

        // yeah, i know this is inefficient. we'll get back to it later.
        PointCloud trimmed_a_current(2, new_n);
        PointCloud trimmed_b(2, new_n);
        for (Eigen::Index i = 0; i < new_n; i++) {
            trimmed_a_current.col(i) = a_current.col(matches[i].point);
            trimmed_b.col(i) = b.col(matches[i].pair);
        }

        Vector trimmed_a_cm = get_centroid(trimmed_a_current);
        Vector trimmed_b_cm = get_centroid(trimmed_b);

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
            V.col(1) *= -1;
            R = V * U.transpose();
        }

        /* #step Transformation Step: see \ref vanilla_icp for details. */
        RBTransform step;
        step.linear() = R;
        step.translation() = trimmed_b_cm - R * trimmed_a_cm;

        transform = step * transform;
    }

    void FeatureAware::compute_matches() {
        Eigen::MatrixXd normalized_dists = compute_norm_dists<2>(a_current, b);
        double max = normalized_dists.maxCoeff();
        if (max > MIN_NORM_THRESHOLD) {
            normalized_dists /= max;
        }

        for (Eigen::Index i = 0; i < a.cols(); i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (Eigen::Index j = 0; j < b.cols(); j++) {
                double dist = normalized_dists(i, j);
                double feature_dist = normalized_feature_dists(i, j);
                double cost = neighbor_weight * dist + feature_weight * feature_dist;

                if (cost < matches[i].cost) {
                    matches[i].cost = cost;
                    matches[i].pair = j;
                }
            }
        }
    }

    Eigen::MatrixXd FeatureAware::compute_features(const PointCloud& points) const {
        Eigen::MatrixXd features(2 * symmetric_neighbors, points.cols());
        features.setZero();

        Vector centroid = get_centroid(points);

        for (Eigen::Index i = 0; i < points.cols(); i++) {
            Vector point1 = points.col(i);
            double cm_dist_p = (point1 - centroid).norm();

            Eigen::Index lower = std::max<Eigen::Index>(0, i - symmetric_neighbors);
            for (Eigen::Index j = lower; j < i; j++) {
                Vector point2 = points.col(j);
                double cm_dist_q = (point2 - centroid).norm();
                features(j - lower, i) = cm_dist_q - cm_dist_p;
            }

            Eigen::Index upper = std::min(points.cols() - 1, i + symmetric_neighbors);
            for (Eigen::Index j = i + 1; j <= upper; j++) {
                Vector point2 = points.col(j);
                double cm_dist_q = (point2 - centroid).norm();
                features(j - i - 1 + symmetric_neighbors, i) = cm_dist_q - cm_dist_p;
            }
        }

        return features;
    }
}

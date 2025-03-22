#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"

/* #name Feature-Aware */
/* #register feature_aware */

/* #desc Builds on top of \ref trimmed_icp. In addition to matching points based on a point-to-point
distance criteria, matches them based on a local "feature vector."
*/

#include "icp/impl/feature_aware.h"

namespace icp {
    FeatureAware::FeatureAware(double overlap_rate, double feature_weight, int symmetric_neighbors)
        : ICP(),
          overlap_rate(overlap_rate),
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
        : FeatureAware(config.get<double>("overlap_rate", 0.9),
              config.get<double>("feature_weight", 0.7),
              config.get<int>("symmetric_neighbors", 10)) {}

    FeatureAware::~FeatureAware() {}

    void FeatureAware::setup() {
        a_current = transform * a;
        matches.resize(a.cols());

        // TODO: is relying on NRVO a good idea?
        a_features = compute_features(a_current);
        b_features = compute_features(b);

        normalized_feature_dists = compute_norm_dists<Eigen::Dynamic>(a_features, b_features);
        normalized_feature_dists /= normalized_feature_dists.maxCoeff();

        // TODO: should we rely on NRVO here?
        compute_matches();
    }

    void FeatureAware::iterate() {
        const size_t n = a.cols();

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

    void FeatureAware::compute_matches() {
        const size_t n = a.cols();
        const size_t m = b.cols();

        Eigen::MatrixXd normalized_dists = compute_norm_dists<2>(a_current, b);
        normalized_dists /= normalized_dists.maxCoeff();

        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
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

    Eigen::MatrixXd FeatureAware::compute_features(const PointCloud& points) {
        const size_t n = points.cols();
        Eigen::MatrixXd features(n, 2 * symmetric_neighbors);
        features.setZero();

        Vector cm = get_centroid<2>(points);

        for (size_t i = 0; i < n; i++) {
            Vector p = points.col(i);
            double cm_dist_p = (p - cm).norm();

            size_t lower = std::max((size_t)0, i - symmetric_neighbors);
            for (size_t j = lower; j < i; j++) {
                Vector q = points.col(j);
                double cm_dist_q = (q - cm).norm();
                features(i, j - lower) = cm_dist_q - cm_dist_p;
            }

            size_t upper = std::min(n - 1, i + symmetric_neighbors);
            for (size_t j = i + 1; j <= upper; j++) {
                Vector q = points.col(j);
                double cm_dist_q = (q - cm).norm();
                features(i, j - i - 1 + symmetric_neighbors) = cm_dist_q - cm_dist_p;
            }
        }

        return features;
    }
}

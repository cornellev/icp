#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

/* #name Feature-Aware */

/* #desc TODO */

#include "icp/impl/feature_aware.h"

namespace icp {
    FeatureAware::FeatureAware(double overlap_rate, double feature_weight, int symmetric_neighbors)
        : ICP(),
          overlap_rate(overlap_rate),
          symmetric_neighbors(symmetric_neighbors),
          feature_weight(feature_weight),
          neighbor_weight(1 - feature_weight) {}

    /* #conf "overlap_rate" A `double` between `0.0` and `1.0` for
     * the overlap rate. The default is `1.0`. */
    /* #conf "feature_weight" A `double` with default value `0.7`. */
    /* #conf "symmetric_neighbors" An `int` with default value `10`. */
    FeatureAware::FeatureAware(const Config& config)
        : FeatureAware(config.get<double>("overlap_rate", 0.9),
              config.get<double>("feature_weight", 0.7),
              config.get<int>("symmetric_neighbors", 10)) {}

    FeatureAware::~FeatureAware() {}

    void FeatureAware::setup() {
        a_current.resize(a.size());
        a_features.resize(a.size());
        b_features.resize(b.size());

        b_cm = get_centroid(b);

        compute_features(a, get_centroid(a), a_features);
        compute_features(b, b_cm, b_features);

        norm_feature_dists = compute_norm_dists(a_features, b_features);

        compute_matches();
    }

    void FeatureAware::iterate() {
        const size_t n = a.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        /* #step  TODO: write smth Matching Step: */
        compute_matches();

        /*
            #step
            Trimming Step: see \ref trimmed for details.
        */
        std::sort(matches.begin(), matches.end(),
            [](const auto& a, const auto& b) { return a.cost < b.cost; });
        size_t new_n = static_cast<size_t>(overlap_rate * n);
        new_n = std::max<size_t>(new_n, 1);  // TODO: bad for scans with 0 points

        // yeah, i know this is inefficient. we'll get back to it later.
        std::vector<icp::Vector> trimmed_current(new_n);
        std::vector<icp::Vector> trimmed_b(new_n);
        for (size_t i = 0; i < new_n; i++) {
            trimmed_current[i] = a_current[matches[i].point];
            trimmed_b[i] = b[matches[i].pair];
        }

        icp::Vector trimmed_cm = get_centroid(trimmed_current);
        icp::Vector trimmed_b_cm = get_centroid(trimmed_b);

        /* #step SVD: see \ref vanilla_icp for details. */
        Matrix N = Matrix::Zero();
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
        transform.translation = R * transform.translation + trimmed_b_cm - R * trimmed_cm;
    }

    void FeatureAware::compute_matches() {
        const size_t n = a.size();
        const size_t m = b.size();

        Eigen::MatrixXd norm_dists = compute_norm_dists(a_current, b);

        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                double dist = norm_dists(i, j);
                double feature_dist = norm_feature_dists(i, j);
                double cost = neighbor_weight * dist + feature_weight * feature_dist;

                if (cost < matches[i].cost) {
                    matches[i].cost = cost;
                    matches[i].pair = j;
                }
            }
        }
    }

    void FeatureAware::compute_features(const std::vector<Vector>& points, Vector cm,
        std::vector<FeatureVector>& features) {
        for (size_t i = 0; i < points.size(); i++) {
            Vector p = points[i];
            double cm_dist_p = (p - cm).norm();

            FeatureVector feature(2 * symmetric_neighbors);
            feature.setZero();

            size_t lower = std::max((size_t)0, i - symmetric_neighbors);
            for (size_t j = lower; j < i; j++) {
                Vector q = points[j];
                double cm_dist_q = (q - cm).norm();
                feature[j - lower] = cm_dist_q - cm_dist_p;
            }

            size_t upper = std::min(points.size() - 1, i + symmetric_neighbors);
            for (size_t j = i + 1; j <= upper; j++) {
                Vector q = points[j];
                double cm_dist_q = (q - cm).norm();
                feature[j - i - 1 + symmetric_neighbors] = cm_dist_q - cm_dist_p;
            }

            features[i] = feature;
        }
    }
}  // namespace icp

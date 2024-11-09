#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "icp/impl/feature_aware.h"

namespace icp {
    FeatureAware::FeatureAware(double feature_weight, int symmetric_neighbors)
        : ICP(),
          symmetric_neighbors(symmetric_neighbors),
          feature_weight(feature_weight),
          neighbor_weight(1 - feature_weight) {}

    FeatureAware::FeatureAware(const Config& config)
        : FeatureAware(config.get<double>("feature_weight", 1.0),
              config.get<int>("symmetric_neighbors", 5)) {}

    FeatureAware::~FeatureAware() {}

    void FeatureAware::setup() {
        a_current.resize(a.size());
        a_features.resize(a.size());
        b_features.resize(b.size());

        b_cm = get_centroid(b);

        compute_features(a, get_centroid(a), a_features);
        compute_features(b, b_cm, b_features);
    }

    void FeatureAware::iterate() {
        const size_t n = a.size();
        const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        auto a_current_cm = get_centroid(a_current);

        /* TODO #step Matching Step: */
        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                // Point-to-point matching
                double dist = (b[j] - a_current[i]).norm();
                double feature_dist = (b_features[j] - a_features[i]).norm();
                double cost = neighbor_weight * dist + feature_weight * feature_dist;

                if (cost < matches[i].cost) {
                    matches[i].cost = cost;
                    matches[i].pair = j;
                }
            }
        }

        // TODO normalize features

        /* #step SVD: see \ref vanilla_icp for details. */
        Matrix N = Matrix::Zero();
        for (size_t i = 0; i < n; i++) {
            N += (a_current[i] - a_current_cm) * (b[matches[i].pair] - b_cm).transpose();
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
        transform.translation += b_cm - R * a_current_cm;
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
}

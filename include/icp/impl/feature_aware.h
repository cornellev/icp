/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include "icp/icp.h"
#include <Eigen/Core>

namespace icp {
    class FeatureAware final : public ICP {
        using FeatureVector = Eigen::VectorXd;

    public:
        FeatureAware(double overlap_rate, double feature_weight, int symmetric_neighbors);
        FeatureAware(const Config& config);
        ~FeatureAware();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();

        void compute_features(const std::vector<icp::Vector>& points, Vector cm,
            std::vector<FeatureVector>& features);

        template<typename TVector>
        Eigen::MatrixXd compute_norm_dists(const std::vector<TVector>& first,
            const std::vector<TVector>& second) {
            Eigen::MatrixXd norm_dists(first.size(), second.size());
            double max_dist = std::numeric_limits<double>::min();
            for (size_t i = 0; i < first.size(); i++) {
                for (size_t j = 0; j < second.size(); j++) {
                    double dist = (first[i] - second[j]).norm();
                    norm_dists(i, j) = dist;
                    if (dist > max_dist) {
                        max_dist = dist;
                    }
                }
            }

            norm_dists /= max_dist;
            return norm_dists;
        }

        std::vector<icp::Vector> a_current;

        icp::Vector b_cm;

        std::vector<FeatureVector> a_features;
        std::vector<FeatureVector> b_features;

        Eigen::MatrixXd norm_feature_dists;

        double overlap_rate;
        int symmetric_neighbors;
        double feature_weight;
        double neighbor_weight;
    };
}

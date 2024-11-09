/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"
#include <Eigen/Core>

namespace icp {
    class FeatureAware final : public ICP {
        using FeatureVector = Eigen::VectorXd;

    public:
        FeatureAware(double feature_weight, int symmetric_neighbors);
        FeatureAware(const Config& config);
        ~FeatureAware();

        void setup() override;
        void iterate() override;

    private:
        void compute_features(const std::vector<icp::Vector>& points, Vector cm,
            std::vector<FeatureVector>& features);

        std::vector<icp::Vector> a_current;

        icp::Vector b_cm;

        std::vector<FeatureVector> a_features;
        std::vector<FeatureVector> b_features;

        int symmetric_neighbors;
        double feature_weight;
        double neighbor_weight;
    };
}

/*
 * @author Utku Melemetci
 */

#include <Eigen/Core>
#include "icp/icp.h"
#include "icp/config.h"

namespace icp {
    class FeatureAware final : public ICP2d {
    public:
        FeatureAware(double overlap_rate, double feature_weight, int symmetric_neighbors);
        FeatureAware(const Config& config);
        ~FeatureAware();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();

        Eigen::MatrixXd compute_features(const PointCloud& points);

        /**
         * @brief Computes a matrix M where M_ij is the distance between the i-th vector (column) of
         * `first` and the j-th vector (column) of `second`. M is of
         * size (first.cols(), second.cols()).
         *
         * @tparam VectorDim the dimension of the vectors
         * @param first the first set of vectors
         * @param second the second set of vectors
         * @return Eigen::MatrixXd the matrix M
         */
        template<const long VectorDim>
        Eigen::MatrixXd compute_norm_dists(
            const Eigen::Matrix<double, VectorDim, Eigen::Dynamic>& first,
            const Eigen::Matrix<double, VectorDim, Eigen::Dynamic>& second) {
            Eigen::MatrixXd dists(first.cols(), second.cols());
            for (size_t i = 0; i < first.cols(); i++) {
                for (size_t j = 0; j < second.cols(); j++) {
                    dists(i, j) = (first.col(i) - second.col(j)).norm();
                }
            }
            return dists;
        }

        PointCloud a_current;

        Eigen::MatrixXd a_features;
        Eigen::MatrixXd b_features;

        Eigen::MatrixXd normalized_feature_dists;

        double overlap_rate;
        int symmetric_neighbors;
        double feature_weight;
        double neighbor_weight;
    };
}

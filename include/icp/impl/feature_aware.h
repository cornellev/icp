/*
 * @author Utku Melemetci
 */

#include <Eigen/Core>
#include <cstddef>
#include "icp/icp.h"
#include "icp/config.h"

namespace icp {
    class FeatureAware final : public ICP2 {
    public:
        FeatureAware(double overlap_rate, double feature_weight, int symmetric_neighbors);
        FeatureAware(const Config& config);
        ~FeatureAware();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();
        void compute_features(const PointCloud& points, Eigen::MatrixXd& features);

        /**
         * @brief Computes a matrix M where M_ij is the distance between the i-th vector (column) of
         * `first` and the j-th vector (column) of `second`. M is of
         * size (first.cols(), second.cols()).
         *
         * @tparam VectorDim the dimension of the vectors
         * @param first The first set of vectors
         * @param second The second set of vectors
         * @param dists The output matrix M. Must be of size (first.cols(), second.cols()).
         */
        template<const long VectorDim>
        void compute_dists(const Eigen::Matrix<double, VectorDim, Eigen::Dynamic>& first,
            const Eigen::Matrix<double, VectorDim, Eigen::Dynamic>& second,
            Eigen::MatrixXd& dists) {
            for (ptrdiff_t j = 0; j < second.cols(); j++) {
                for (ptrdiff_t i = 0; i < first.cols(); i++) {
                    dists(i, j) = (first.col(i) - second.col(j)).norm();
                }
            }
        }

        double overlap_rate;
        int symmetric_neighbors;
        double feature_weight;
        double neighbor_weight;

        PointCloud a_current;

        PointCloud trimmed_a_current;
        PointCloud trimmed_b;

        Eigen::MatrixXd a_features;
        Eigen::MatrixXd b_features;

        Eigen::MatrixXd normalized_dists;
        Eigen::MatrixXd normalized_feature_dists;
    };
}

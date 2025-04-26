/*
 *
 */
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "icp/impl/trimmed_3d.h"
#include "algo/kdtree.h"
#include "icp/geo.h"

/* #name Trimmed */

namespace icp {
    Trimmed_3d::Trimmed_3d([[maybe_unused]] const Config& config)
        : ICP(),
          c(3, 0),
          current_cost_(std::numeric_limits<double>::max()),
          max_distance(config.get<double>("max_distance", 1.0)) {}
    Trimmed_3d::Trimmed_3d(): ICP(), c(3, 0), current_cost_(std::numeric_limits<double>::max()) {}
    Trimmed_3d::~Trimmed_3d() {}

    // Euclidean distance between two points
    float Trimmed_3d::dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
        return (pta - ptb).norm();
    }

    NEIGHBOR Trimmed_3d::nearest_neighbor(const PointCloud& src, const PointCloud& dst) {
        NEIGHBOR neigh;
        neigh.distances.resize(src.size());
        neigh.indices.resize(src.size());

        // Build KDTree
        std::vector<Eigen::Vector3d> dst_vec(dst.cols());
        for (ptrdiff_t i = 0; i < dst.cols(); ++i) {
            dst_vec[i] = dst.col(i);
        }
        icp::KdTree<Eigen::Vector3d> kdtree(dst_vec, 3);

        for (Eigen::Index i = 0; i < src.cols(); ++i) {
            const Eigen::Vector3d query = src.col(i);
            double min_dist = 0.0;
            int idx = kdtree.search(query, &min_dist);

            neigh.indices[i] = idx;
            neigh.distances[i] = std::sqrt(min_dist);
        }

        return neigh;
    }

    Trimmed_3d::RBTransform Trimmed_3d::best_fit_transform(const PointCloud& A,
        const PointCloud& B) {
        Vector centroid_A = get_centroid(A);
        Vector centroid_B = get_centroid(B);

        Eigen::Matrix3d N = (A.colwise() - centroid_A) * (B.colwise() - centroid_B).transpose();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(N, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }

        Vector t = centroid_B - R * centroid_A;

        RBTransform transform;
        transform.linear() = R;
        transform.translation() = t;

        return transform;
    }

    void Trimmed_3d::setup() {
        c = a;
        current_cost_ = std::numeric_limits<double>::max();
    }

    void Trimmed_3d::iterate() {
        // Reorder target point set based on nearest neighbor
        NEIGHBOR neighbor = nearest_neighbor(c, b);
        PointCloud dst_reordered(3, a.cols());  // Assuming PointCloud is a 3xN matrix
        std::vector<Eigen::Vector3d>
            src_valid;  // Get the valid points from source and distance according to max_distance
        std::vector<Eigen::Vector3d> dst_valid;

        for (ptrdiff_t i = 0; i < a.cols(); ++i) {
            if (neighbor.distances[i] <= max_distance) {
                src_valid.push_back(c.col(i));
                dst_valid.push_back(b.col(neighbor.indices[i]));
            } else {
                std::cerr << "Distance " << neighbor.distances[i] << " exceeds max_distance "
                          << max_distance << std::endl;
            }
        }

        Eigen::MatrixXd AA(3, src_valid.size());
        Eigen::MatrixXd BB(3, dst_valid.size());
        for (size_t i = 0; i < src_valid.size(); ++i) {
            AA.col(i) = src_valid[i];
            BB.col(i) = dst_valid[i];
        }

        RBTransform T = best_fit_transform(AA, BB);
        c = T * c;
        transform = T * transform;

        calculate_cost(neighbor.distances);
    }

    void Trimmed_3d::calculate_cost(const std::vector<float>& distances) {
        if (distances.empty()) {
            current_cost_ = std::numeric_limits<double>::max();
            return;
        }

        double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
        current_cost_ = sum / distances.size();
    }
}

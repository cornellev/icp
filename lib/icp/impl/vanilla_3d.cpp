/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include <cstddef>
#include <numeric>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "icp/impl/vanilla_3d.h"
#include "algo/kdtree.h"
#include "icp/geo.h"

namespace icp {
    Vanilla3d::Vanilla3d([[maybe_unused]] const Config& config)
        : ICP(), c(3, 0), current_cost_(std::numeric_limits<double>::max()) {}
    Vanilla3d::Vanilla3d(): ICP(), c(3, 0), current_cost_(std::numeric_limits<double>::max()) {}
    Vanilla3d::~Vanilla3d() {}

    // Euclidean distance between two points
    float Vanilla3d::dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
        return (pta - ptb).norm();
    }

    Neighbors Vanilla3d::nearest_neighbor(const PointCloud& src, const PointCloud& dst) {
        Neighbors neigh;
        neigh.distances.resize(src.cols());
        neigh.indices.resize(src.cols());

        for (Eigen::Index i = 0; i < src.cols(); ++i) {
            const Eigen::Vector3d query = src.col(i);
            double min_dist = 0.0;
            int idx = kdtree_->search(query, &min_dist);

            neigh.indices[i] = idx;
            neigh.distances[i] = std::sqrt(min_dist);
        }

        return neigh;
    }

    Vanilla3d::RBTransform Vanilla3d::best_fit_transform(const PointCloud& A, const PointCloud& B) {
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

    void Vanilla3d::setup() {
        c = a;
        current_cost_ = std::numeric_limits<double>::max();

        std::vector<Eigen::Vector3d> dst_vec(b.cols());
        for (ptrdiff_t i = 0; i < b.cols(); ++i) {
            dst_vec[i] = b.col(i);
        }

        kdtree_ = std::make_unique<icp::KdTree<Eigen::Vector3d>>(dst_vec, 3);
    }

    void Vanilla3d::iterate() {
        // Reorder target point set based on nearest neighbor
        Neighbors neighbor = nearest_neighbor(c, b);
        PointCloud dst_reordered(3, a.cols());  // Assuming PointCloud is a 3xN matrix
        for (ptrdiff_t i = 0; i < a.cols(); i++) {
            dst_reordered.col(i) = b.col(neighbor.indices[i]);
        }
        RBTransform T = best_fit_transform(c, dst_reordered);
        c = T * c;

        transform = T * transform;

        calculate_cost(neighbor.distances);
    }

    void Vanilla3d::calculate_cost(const std::vector<float>& distances) {
        if (distances.empty()) {
            current_cost_ = std::numeric_limits<double>::max();
            return;
        }

        double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
        current_cost_ = sum / distances.size();
    }
}

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

#include "icp/impl/vanilla_3d.h"
#include "algo/kdtree.h"
#include "icp/geo.h"

/* #name Vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
    Vanilla_3d::Vanilla_3d([[maybe_unused]] const Config& config)
        : ICP(), current_cost_(std::numeric_limits<double>::max()) {}
    Vanilla_3d::Vanilla_3d()
        : ICP(), target_kdtree_(nullptr), current_cost_(std::numeric_limits<double>::max()) {}
    Vanilla_3d::~Vanilla_3d() {}

    void Vanilla_3d::rebuild_kdtree() {
        // TODO: kdtree should take point cloud
        std::vector<Vector> b_vec(b.cols());
        for (ptrdiff_t i = 0; i < b.cols(); i++) {
            b_vec[i] = b.col(i);
        }
        target_kdtree_ = std::make_unique<KdTree<Vector>>(b_vec, 4);
    }

    // Euclidean distance between two points
    float Vanilla_3d::dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
        return (pta - ptb).norm();
    }

    NEIGHBOR Vanilla_3d::nearest_neighbor(const PointCloud& src, const PointCloud& dst) {
        NEIGHBOR neigh;
        neigh.distances.resize(src.cols());
        neigh.indices.resize(dst.cols());

        // For small point clouds or when testing rotation with few points,
        // linear search can give more precise results
        // TODO: why :skull:
        if (src.cols() <= 3 || dst.cols() <= 3) {
            for (ptrdiff_t i = 0; i < src.cols(); i++) {
                Eigen::Vector3d pta = src.col(i);
                float min_dist = std::numeric_limits<float>::max();
                ptrdiff_t index = 0;

                for (ptrdiff_t j = 0; j < dst.cols(); j++) {
                    Eigen::Vector3d ptb = dst.col(j);
                    float d = dist(pta, ptb);
                    if (d < min_dist) {
                        min_dist = d;
                        index = j;
                    }
                }

                neigh.distances[i] = min_dist;
                neigh.indices[i] = index;
            }
            return neigh;
        }

        if (target_kdtree_) {
            bool kdtree_failed = false;

            for (ptrdiff_t i = 0; i < src.cols(); i++) {
                try {
                    Vector query_point = Vector::Zero();
                    query_point(0) = src(i, 0);
                    query_point(1) = src(i, 1);
                    query_point(2) = src(i, 2);

                    float min_dist = 0;
                    ptrdiff_t nearest_idx = target_kdtree_->find_nearest(query_point, &min_dist);

                    if (nearest_idx < dst.cols()) {
                        neigh.indices[i] = nearest_idx;
                        neigh.distances[i] = std::sqrt(min_dist);
                    } else {
                        kdtree_failed = true;
                        break;
                    }
                } catch (...) {
                    kdtree_failed = true;
                    break;
                }
            }

            if (!kdtree_failed) {
                return neigh;
            }

            std::cerr << "KdTree search failed, falling back to linear search" << std::endl;
        }

        // Fall back to linear search if KD-tree fails or is not available
        neigh.distances.clear();
        neigh.indices.clear();
        neigh.distances.resize(src.cols());
        neigh.indices.resize(src.cols());

        for (ptrdiff_t i = 0; i < src.cols(); i++) {
            Vector pta = src.col(i);
            float min_dist = std::numeric_limits<float>::max();
            ptrdiff_t index = 0;

            for (ptrdiff_t j = 0; j < dst.cols(); j++) {
                Vector ptb = dst.col(j);
                float d = dist(pta, ptb);
                if (d < min_dist) {
                    min_dist = d;
                    index = j;
                }
            }

            neigh.distances[i] = min_dist;
            neigh.indices[i] = index;
        }

        return neigh;
    }

    Vanilla_3d::RBTransform Vanilla_3d::best_fit_transform(const PointCloud& A,
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

    void Vanilla_3d::setup() {
        c = a;

        if (!target_kdtree_ && b.cols() != 0) {
            rebuild_kdtree();
        }
        current_cost_ = std::numeric_limits<double>::max();
    }

    void Vanilla_3d::iterate() {
        // Reorder target point set based on nearest neighbor
        NEIGHBOR neighbor = nearest_neighbor(c, b);
        PointCloud dst_reordered(3, a.cols());  // Assuming PointCloud is a 3xN matrix
        for (ptrdiff_t i = 0; i < a.cols(); i++) {
            dst_reordered.col(i) = b.col(neighbor.indices[i]);
        }
        RBTransform T = best_fit_transform(c, dst_reordered);
        c = T * c;

        transform = T;

        calculate_cost(neighbor.distances);
    }

    void Vanilla_3d::calculate_cost(const std::vector<float>& distances) {
        if (distances.empty()) {
            current_cost_ = std::numeric_limits<double>::max();
            return;
        }

        double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
        current_cost_ = sum / distances.size();
    }
}
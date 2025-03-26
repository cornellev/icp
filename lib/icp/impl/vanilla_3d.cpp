/*
 *
 */
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

/* #name Vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
    Vanilla_3d::Vanilla_3d([[maybe_unused]] const Config& config)
        : ICP(3), current_cost_(std::numeric_limits<double>::max()) {}
    Vanilla_3d::Vanilla_3d()
        : ICP(3), target_kdtree_(nullptr), current_cost_(std::numeric_limits<double>::max()) {}
    Vanilla_3d::~Vanilla_3d() {}

    void Vanilla_3d::set_target(const std::vector<Vector>& target) {
        ICP::set_target(target);
        rebuild_kdtree();
    }

    void Vanilla_3d::rebuild_kdtree() {
        if (!b.empty()) {
            try {
                target_kdtree_ = std::make_unique<KdTree<Vector>>(b, 10);
            } catch (const std::exception& e) {
                std::cerr << "Error building KdTree: " << e.what() << std::endl;
                target_kdtree_ = nullptr;
            }
        } else {
            target_kdtree_ = nullptr;
        }
    }

    // Euclidean distance between two points
    float Vanilla_3d::dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
        return (pta - ptb).norm();
    }

    NEIGHBOR Vanilla_3d::nearest_neighbor(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst) {
        size_t row_src = src.rows();
        size_t row_dst = dst.rows();
        NEIGHBOR neigh;
        neigh.distances.resize(row_src);
        neigh.indices.resize(row_src);

        // For small point clouds or when testing rotation with few points,
        // linear search can give more precise results
        // TODO: why :skull:
        if (row_src <= 3 || row_dst <= 3) {
            for (size_t i = 0; i < row_src; i++) {
                Eigen::Vector3d pta = src.row(i).transpose();
                float min_dist = std::numeric_limits<float>::max();
                size_t index = 0;

                for (size_t j = 0; j < row_dst; j++) {
                    Eigen::Vector3d ptb = dst.row(j).transpose();
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

            for (size_t i = 0; i < row_src; i++) {
                try {
                    Vector query_point = Vector::Zero(3);
                    query_point(0) = src(i, 0);
                    query_point(1) = src(i, 1);
                    query_point(2) = src(i, 2);

                    float min_dist = 0;
                    size_t nearest_idx = target_kdtree_->find_nearest(query_point, &min_dist);

                    if (nearest_idx < row_dst) {
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
        neigh.distances.resize(row_src);
        neigh.indices.resize(row_src);

        for (size_t i = 0; i < row_src; i++) {
            Eigen::Vector3d pta = src.row(i).transpose();
            float min_dist = std::numeric_limits<float>::max();
            size_t index = 0;

            for (size_t j = 0; j < row_dst; j++) {
                Eigen::Vector3d ptb = dst.row(j).transpose();
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

    Eigen::Matrix4d Vanilla_3d::best_fit_transform(const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B) {
        Eigen::Vector3d centroid_A = A.colwise().mean();
        Eigen::Vector3d centroid_B = B.colwise().mean();

        Eigen::MatrixXd AA = A.rowwise() - centroid_A.transpose();
        Eigen::MatrixXd BB = B.rowwise() - centroid_B.transpose();

        Eigen::Matrix3d H = AA.transpose() * BB;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }

        Eigen::Vector3d t = centroid_B - R * centroid_A;

        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(A.cols() + 1, A.cols() + 1);

        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        return T;
    }

    void Vanilla_3d::setup() {
        A.resize(a.size(), dim);
        for (size_t i = 0; i < a.size(); ++i) {
            for (size_t j = 0; j < dim; ++j) {
                A(i, j) = a[i][j];
            }
        }

        B.resize(b.size(), dim);
        for (size_t i = 0; i < b.size(); ++i) {
            for (size_t j = 0; j < dim; ++j) {
                B(i, j) = b[i][j];
            }
        }

        C = A;

        if (!target_kdtree_ && !b.empty()) {
            rebuild_kdtree();
        }
        current_cost_ = std::numeric_limits<double>::max();
    }

    void Vanilla_3d::iterate() {
        size_t row = A.rows();

        // Reorder target point set based on nearest neighbor
        NEIGHBOR neighbor = nearest_neighbor(C, B);
        Eigen::MatrixXd dst_reordered(row, 3);
        for (size_t i = 0; i < row; i++) {
            dst_reordered.row(i) = B.row(neighbor.indices[i]);
        }
        Eigen::Matrix4d T = best_fit_transform(C, dst_reordered);
        Eigen::MatrixXd C_homogeneous = C.transpose().colwise().homogeneous();
        C = (T * C_homogeneous).transpose().leftCols(3);

        RBTransform step(T.block<3, 1>(0, 3), T.block<3, 3>(0, 0));
        transform = transform.and_then(step);

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

    double Vanilla_3d::get_current_cost() const {
        return current_cost_;
    }
}
/*
 *
 */

#include <numeric>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "icp/impl/vanilla_3d.h"

/* #name Vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
    Vanilla_3d::Vanilla_3d([[maybe_unused]] const Config& config): ICP(3) {}
    Vanilla_3d::Vanilla_3d(): ICP(3) {}
    Vanilla_3d::~Vanilla_3d() {}

    // Euclidean distance between two points
    float Vanilla_3d::dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
        return (pta - ptb).norm();
    }

    // Find the nearest neighbor
    NEIGHBOR Vanilla_3d::nearest_neighbor(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst) {
        int row_src = src.rows();
        int row_dst = dst.rows();
        NEIGHBOR neigh;

        for (int i = 0; i < row_src; i++) {
            Eigen::Vector3d pta = src.row(i).transpose();
            float min_dist = std::numeric_limits<float>::max();
            int index = 0;

            for (int j = 0; j < row_dst; j++) {
                Eigen::Vector3d ptb = dst.row(j).transpose();
                float d = dist(pta, ptb);
                if (d < min_dist) {
                    min_dist = d;
                    index = j;
                }
            }

            neigh.distances.push_back(min_dist);
            neigh.indices.push_back(index);
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
        T.block<3, 1>(0, A.cols()) = t;
        return T;
    }

    void Vanilla_3d::setup() {
        C = A;
    }

    void Vanilla_3d::iterate() {
        int row = A.rows();

        // Reorder target point set based on nearest neighbor
        NEIGHBOR neighbor = nearest_neighbor(C, B);
        Eigen::MatrixXd dst_reordered(row, 3);
        for (int i = 0; i < row; i++) {
            dst_reordered.row(i) = B.row(neighbor.indices[i]);
        }

        Eigen::Matrix4d T = best_fit_transform(C, dst_reordered);
        C = (T * C.transpose().colwise().homogeneous()).transpose();

        transform = transform.update(T);
    }
}
/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */
#include <iostream>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <vector>
#include "icp/geo.h"

#include "icp/impl/vanilla.h"

/* #name Vanilla */
/* #register vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
    Vanilla::Vanilla([[maybe_unused]] const Config& config): ICP(2) {}
    Vanilla::Vanilla(): ICP(2) {}
    Vanilla::~Vanilla() {}

    void Vanilla::setup() {
        a_current.resize(a.size());//make function in icp
        // b_cm = get_centroid(b);

        for (size_t i = 0; i < a.size(); i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        compute_matches();

        icp::Vector corr_cm = icp::Vector::Zero(2);
        for (size_t i = 0; i < matches.size(); i++) {
            corr_cm += b[matches[i].pair];
        }
        corr_cm /= matches.size();
        b_cm = corr_cm;
    }

    void Vanilla::iterate() {
        const size_t n = a.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        // can optimize by just transforming prev centroid if necessary
        auto a_current_cm = get_centroid(a_current);

        /*
            #step
            Matching Step: match closest points.

            Sources:
            https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
            https://arxiv.org/pdf/2206.06435.pdf
            https://web.archive.org/web/20220615080318/https://www.cs.technion.ac.il/~cs236329/tutorials/ICP.pdf
            https://en.wikipedia.org/wiki/Iterative_closest_point
            https://courses.cs.duke.edu/spring07/cps296.2/scribe_notes/lecture24.pdf
            -> use k-d tree
         */
        compute_matches();

        /*
            #step
            SVD

            We compute the SVD of this magical matrix. A proof that this yields the optimal
            transform R is in the source below.

            Sources:
            https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
        */
        Matrix N = Matrix::Zero(2,2);
        for (size_t i = 0; i < n; i++) {
            N += (a_current[i] - a_current_cm) * (b[matches[i].pair] - b_cm).transpose();
        }
        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();
        std::cout << "0: ___" << std::endl;
        std::cout << R << std::endl;

        /*
            #step
            Reflection Handling

            SVD may return a reflection instead of a rotation if it's equally good or better.
            This is exceedingly rare with real data but may happen in very high noise
            environment with sparse point cloud.

            In the 2D case, we can always recover a reasonable rotation by negating the last
            column of V. I do not know if this is the optimal rotation among rotations, but
            we can probably get an answer to that question with more effort.

            Sources:
            https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
        */
        if (R.determinant() < 0) {
            std::cout << "1: ___" << std::endl;
            std::cout << R << std::endl;
            std::cout << "Utrans: " << U.transpose() << std::endl;
            std::cout << "Determinant: " << R.determinant() << std::endl;
            std::cout << "V: " << V << std::endl;
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            std::cout << "V: " << V << std::endl;
            R = V * U.transpose();
            std::cout << "2: ___" << std::endl;
            std::cout << R << std::endl;
        }
        std::cout << "3: ___" << std::endl;
        std::cout << get_matches()[0].pair << std::endl;
        std::cout << get_matches()[1].pair << std::endl;

        //transform.rotation = R * transform.rotation;

        /*
           #step
           Transformation Step: determine optimal transformation.

           The translation vector is determined by the displacement between
           the centroids of both point clouds. The rotation matrix is
           calculated via singular value decomposition.

            Sources:
            https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
            https://courses.cs.duke.edu/spring07/cps296.2/scribe_notes/lecture24.pdf
         */
        //transform.translation = R * transform.translation + b_cm - R * a_current_cm;

        Vector translation_update = b_cm - R * a_current_cm;
        RBTransform update_transform(translation_update, R);
        transform = transform.and_then(update_transform);
    }

    void Vanilla::compute_matches() {
        const size_t n = a.size();
        const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                // Point-to-point matching
                double dist_ij = (b[j] - a_current[i]).squaredNorm();

                if (dist_ij < matches[i].cost) {
                    matches[i].cost = dist_ij;
                    matches[i].pair = j;
                }
            }
        }
    }
}
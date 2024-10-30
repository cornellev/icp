/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include "../icp.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

/* #name Vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
    struct Vanilla final : public ICP {
        std::vector<icp::Vector> a_current;
        icp::Vector b_cm;

        Vanilla(): ICP() {}
        ~Vanilla() override {}

        void setup() override {
            if (a_current.size() < a.size()) {
                a_current.resize(a.size());
            }

            b_cm = get_centroid(b);
        }

        void iterate() override {
            const size_t n = a.size();
            const size_t m = b.size();

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
            for (size_t i = 0; i < n; i++) {
                matches[i].sq_dist = std::numeric_limits<double>::infinity();
                for (size_t j = 0; j < m; j++) {
                    // Point-to-point matching
                    double dist_ij = (b[j] - a_current[i]).squaredNorm();

                    if (dist_ij < matches[i].sq_dist) {
                        matches[i].sq_dist = dist_ij;
                        matches[i].pair = j;
                    }
                }
            }

            /*
                #step
                SVD

                We compute the SVD of this magical matrix. A proof that this yields the optimal
                transform R is in the source below.

                Sources:
                https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
            */
            Matrix N{};
            for (size_t i = 0; i < n; i++) {
                N += (a_current[i] - a_current_cm) * (b[matches[i].pair] - b_cm).transpose();
            }
            auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
            const Matrix U = svd.matrixU();
            Matrix V = svd.matrixV();
            Matrix R = V * U.transpose();

            /*
                #step
                Reflection Handling

                SVD may return a reflection instead of a rotation if it's equally good or better.
                This is exceedingly rare with real data but may happen in very high noise
                environment with sparse point cloud.

                In the 2D case, we can always recover a reasonable rotation by negating the last
                column of V. I do not know if this is the optimal rotation among rotations, but
                we can probably get an answer with more effort.

                Sources:
                https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
            */
            if (R.determinant() < 0) {
                V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
                R = V * U.transpose();
            }

            transform.rotation = R * transform.rotation;

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
            transform.translation += b_cm - R * a_current_cm;
        }
    };

    static bool static_initialization = []() {
        assert(ICP::register_method("vanilla",
            []([[maybe_unused]] const ICP::Config& config) -> std::unique_ptr<ICP> {
                return std::make_unique<Vanilla>();
            }));
        return true;
    }();
}

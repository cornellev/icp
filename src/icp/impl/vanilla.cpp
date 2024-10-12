/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include "../icp.h"
#include <Eigen/Core>
#include <Eigen/SVD>

/* #name Vanilla */

/* #desc The vanilla algorithm for ICP will match the point-cloud centers
exactly and then iterate until an optimal rotation has been found. */

namespace icp {
struct Vanilla final : public ICP {
    std::vector<icp::Vector> a_rot;

    Vanilla(): ICP() {}
    ~Vanilla() override {}

    void setup() override {
        if (a_rot.size() < a.size()) {
            a_rot.resize(a.size());
        }
    }

    void iterate() override {
        const size_t n = a.size();
        const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            a_rot[i] = transform.rotation * a[i];
        }

        /*
            #step
            Matching Step: match closest points.

            Sources:
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
                double dist_ij = (b[j] - a_rot[i]).squaredNorm();

                if (dist_ij < matches[i].sq_dist) {
                    matches[i].sq_dist = dist_ij;
                    matches[i].pair = j;
                }
            }
        }

        /*
            #step
            Transformation Step: determine optimal transformation.

            The translation vector is determined by the displacement between
            the centroids of both point clouds. The rotation matrix is
            calculated via singular value decomposition.

            Sources:
            https://courses.cs.duke.edu/spring07/cps296.2/scribe_notes/lecture24.pdf
         */
        transform.translation = b_cm - transform.rotation * a_cm;

        Matrix N{};
        for (size_t i = 0; i < n; i++) {
            N += a[i] * b[matches[i].pair].transpose();
        }
        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Matrix U = svd.matrixU();
        const Matrix V = svd.matrixV();
        transform.rotation = V * U.transpose();
    }
};

static bool static_initialization = []() {
    assert(ICP::register_method("vanilla", [](const ICP::Config& config) -> std::unique_ptr<ICP> {
        return std::make_unique<Vanilla>();
    }));
    return true;
}();
}  // namespace icp

/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include "../icp.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

/* #name Trimmed */

/* #desc Trimmed ICP is identical to \ref vanilla_icp with the addition of an
overlap rate parameter, which specifies the percentage of points between the two
point sets that have correspondences. When the overlap rate is 1, the algorithm
reduces to vanilla. */

namespace icp {
    struct Trimmed final : public ICP {
        double overlap_rate;
        std::vector<icp::Vector> a_current;
        icp::Vector b_cm;

        Trimmed(double overlap_rate): ICP(), overlap_rate(overlap_rate) {}
        ~Trimmed() override {}

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

            /* #step Matching Step: see \ref vanilla_icp for details. */
            for (size_t i = 0; i < n; i++) {
                matches[i].point = i;
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
                Trimming Step

                Matches are considered in increasing order of distance.

                Sources:
                https://ieeexplore.ieee.org/abstract/document/1047997
            */
            std::sort(matches.begin(), matches.end(),
                [](const auto& a, const auto& b) { return a.sq_dist < b.sq_dist; });
            size_t new_n = (size_t)(overlap_rate * n);

            // yeah, i know this is inefficient. we'll get back to it later.
            std::vector<icp::Vector> trimmed_current(new_n);
            std::vector<icp::Vector> trimmed_b(new_n);
            for (size_t i = 0; i < new_n; i++) {
                trimmed_current[i] = a_current[matches[i].point];
                trimmed_b[i] = b[matches[i].point];
            }

            icp::Vector trimmed_cm = get_centroid(trimmed_current);
            icp::Vector trimmed_b_cm = get_centroid(trimmed_b);

            Matrix N{};
            for (size_t i = 0; i < new_n; i++) {
                N += (trimmed_current[i] - trimmed_cm) * (trimmed_b[i] - trimmed_b_cm).transpose();
            }

            auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
            Matrix U = svd.matrixU();
            Matrix V = svd.matrixV();
            Matrix R = V * U.transpose();

            /*
                #step
                Reflection Handling

                SVD may return a reflection instead of a rotation. For 2D scans, this case is
                exceedingly rare because it only happens when our "a" scan is colinear, as far as I
                can tell (at least in a noiseless case). If this happens in a noisy case and the
                points aren't somewhat colinear, I don't know of a method to recover a good
                rotation. So we will just assume little noise and guess that the scan is colinear.
                The source below claims that the least squares solution is probably useless in a
                similar case in 3D.

                TODO: This is the 2D translation of what the source does. Can we prove that this
                actually works?

                Sources:
                https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
            */
            if (R.determinant() < 0) {
                // clearly average angle is a wrong idea lol
                std::cout << "Reflection" << std::endl;
                std::cout << "Matrix V: " << V << std::endl;
                std::cout << "Matrix U: " << U << std::endl;
                std::cout << "Sing vals: " << svd.singularValues() << std::endl;

                V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
                R = V * U.transpose();
            }

            transform.rotation = R * transform.rotation;

            /* #step Transformation Step: see \ref vanilla_icp for details. */
            transform.translation += trimmed_b_cm - R * trimmed_cm;
        }
    };

    static bool static_initialization = []() {
        assert(ICP::register_method("trimmed",
            [](const ICP::Config& config) -> std::unique_ptr<ICP> {
                /* #conf "overlap_rate" A `double` between `0.0` and `1.0` for
                 * the overlap rate. The default is `1.0`. */
                double overlap_rate = config.get<double>("overlap_rate", 1.0);
                assert(overlap_rate >= 0 && overlap_rate <= 1);
                return std::make_unique<Trimmed>(overlap_rate);
            }));
        return true;
    }();
}

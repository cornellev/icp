/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include "../icp.h"
#include <Eigen/Core>
#include <Eigen/SVD>

/* #name Test1 */
/* #desc This is a WIP. */

namespace icp {
    struct Test1 final : public ICP {
        double overlap_rate;
        std::vector<icp::Vector> a_rot;

        Test1(double overlap_rate): ICP(), overlap_rate(overlap_rate) {}
        ~Test1() override {}

        void setup() override {
            if (a_rot.size() < a.size()) {
                a_rot.resize(a.size());
            }
        }

        void iterate() override {
            size_t n = a.size();
            const size_t m = b.size();

            for (size_t i = 0; i < n; i++) {
                a_rot[i] = transform.rotation * a[i];
            }

            /* #step Matching Step: see \ref vanilla_icp
            for details. */
            for (size_t i = 0; i < n; i++) {
                matches[i].point = i;
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
                #step Trimming Step: see \ref trimmed_icp for details.
            */
            std::sort(matches.begin(), matches.end(),
                [](const auto& a, const auto& b) { return a.sq_dist < b.sq_dist; });
            n = (size_t)(overlap_rate * n);

            /*
                #step
                Transformation Step

                See [this paper](icp.pdf) for information. I copied over the
               rotation optimization from \ref vanilla_icp, and it is a TODO to
               see if that is mathematically correct or not.
             */

            transform.translation = Vector::Zero();
            for (size_t i = 0; i < n; i++) {
                transform.translation += (b[matches[i].pair] + b_cm)
                                         - transform.rotation * (a[matches[i].point] + a_cm);
            }
            transform.translation /= n;

            // no clue why this might still work
            // TODO: mathematically see if you can justify it, otherwise scrap
            // and find new method
            Matrix N{};
            for (size_t i = 0; i < n; i++) {
                N += (a[matches[i].point] + transform.translation) * b[matches[i].pair].transpose();
            }
            auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
            const Matrix U = svd.matrixU();
            const Matrix V = svd.matrixV();
            transform.rotation = V * U.transpose();
        }
    };

    static bool static_initialization = []() {
        assert(ICP::register_method("test1", [](const ICP::Config& config) -> std::unique_ptr<ICP> {
            /* #conf "overlap_rate" A `double` between `0.0` and `1.0` for
             * the overlap rate. The default is `1.0`. */
            double overlap_rate = config.get<double>("overlap_rate", 1.0);
            assert(overlap_rate >= 0 && overlap_rate <= 1);
            return std::make_unique<Test1>(overlap_rate);
        }));
        return true;
    }();
}

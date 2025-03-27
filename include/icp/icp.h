/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include "geo.h"

namespace icp {
    /**
     * Interface for iterative closest points.
     * Generally, you should interact with ICP instances through this interface or `ICPDriver`,
     * though interacting with implementations directly isn't explicitly disallowed.
     * Read \ref write_icp_instance for additional information.
     *
     * \par Example
     * @code
     * // Construct a new vanilla ICP instance.
     * std::unique_ptr<icp::ICP> icp = icp::ICP::from_method("vanilla");
     * @endcode
     *
     * \par Usage
     * Let `a` and `b` be two point clouds of type `std::vector<Vector>`.
     * Then, given an ICP instance `icp` of type `std::unique_ptr<icp::ICP>`,
     * perform the following steps.
     *
     * 1. Call `icp->begin(a, b, initial_guess)`.
     * 2. Repeatedly call `icp->iterate()` until convergence. `ICPDriver` can also be used to
     * specify convergence conditions.
     *
     * If these steps are not followed as described here, the behavior is
     * undefined.
     *
     * At any point in the process, you may query `icp->calculate_cost()` and
     * `icp->transform()`. Do note, however, that `icp->calculate_cost()` is not
     * a constant-time operation.
     */
    template<const int Dim>
    class ICP {
    protected:
        using Vector = icp::Vector<Dim>;
        using RBTransform = icp::RBTransform<Dim>;
        using PointCloud = icp::PointCloud<Dim>;

    protected:
        /** A matching between `point` and `pair` at (arbitrary) cost `cost`.  */
        struct Match {
            size_t point;
            size_t pair;
            double cost;
        };

        /** The current point cloud transformation that is being optimized. */
        RBTransform transform;

        /** The source point cloud. */
        PointCloud a;

        /** The destination point cloud. */
        PointCloud b;

        /** The pairing of each point in `a` to its closest in `b`. */
        std::vector<Match> matches;

        ICP() {}

        /**
         * @brief Per-method setup code.
         *
         * @post For implementers: must fill `matches` with match data for the initial point clouds.
         */
        virtual void setup() = 0;

    public:
        virtual ~ICP() = default;

        /** Begins the ICP process for point clouds `a` and `b` with an initial
         * guess for the transform `t`.*/
        void begin(const PointCloud& a, const PointCloud& b, const RBTransform& t) {
            // Initial transform guess
            this->transform = t;

            // Copy in point clouds
            this->a = a;
            this->b = b;

            // Ensure arrays are the right size
            matches.resize(this->a.cols());

            // Per-instance customization routine
            setup();
        }

        /** Perform one iteration of ICP for the point clouds `a` and `b`
         * provided with ICP::begin.
         *
         * @pre ICP::begin must have been invoked.
         * @post For implementers: must fill `matches` with newest match data.
         */
        virtual void iterate() = 0;

        /**
         * Computes the cost of the current transformation.
         *
         * \par Efficiency:
         * `O(a.size())` where `a` is the source point cloud.
         */
        double calculate_cost() const {
            double sum_squares = 0.0;
            for (auto& match: matches) {
                sum_squares += match.cost;
            }
            return std::sqrt(sum_squares / a.cols());
        }

        /** The current transform. */
        RBTransform current_transform() const {
            return transform;
        }
    };

    using ICP3 = ICP<3>;
    using ICP2 = ICP<2>;
}
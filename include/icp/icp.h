/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <optional>
#include <Eigen/Core>

#include "geo.h"
#include "config.h"

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
    template<const Dimension Dim>
    class ICP {
    private:
        using MethodConstructor = std::function<std::unique_ptr<ICP<Dim>>(const Config&)>;
        static std::unordered_map<std::string, MethodConstructor> methods;

    protected:
        using Vector = icp::Vector<Dim>;
        using RBTransform = icp::RBTransform<Dim>;
        using PointCloud = icp::PointCloud<Dim>;

        /** A matching between `point` and `pair` at (arbitrary) cost `cost`.  */
        struct Match {
            /** An index into the source point cloud. */
            Eigen::Index point;

            /** An index into the destination point cloud. */
            Eigen::Index pair;

            /** The (arbitrary) cost of the pair. */
            double cost;
        };

        /** The current point cloud transformation that is being optimized. */
        RBTransform transform = RBTransform::Identity();

        /** The source point cloud. */
        PointCloud a;

        /** The destination point cloud. */
        PointCloud b;

        /** The pairing of each point in `a` to its closest in `b`. */
        std::vector<Match> matches;

        ICP(): a(Dim, 0), b(Dim, 0) {}

        /**
         * @brief Per-method setup code.
         *
         * @post For implementers: must fill `matches` with match data for the initial point
         * clouds.
         */
        virtual void setup() = 0;

    public:
        static std::optional<std::unique_ptr<ICP<Dim>>> from_method(const std::string& name,
            const Config& config) {
            if (methods.find(name) == methods.end()) {
                return {};
            }

            return methods[name](config);
        }

        static bool is_method_registered(const std::string& name) {
            return methods.find(name) != methods.end();
        }

        static std::vector<std::string> registered_methods() {
            std::vector<std::string> keys;
            for (auto it = methods.begin(); it != methods.end(); ++it) {
                keys.push_back(it->first);
            }
            return keys;
        }

        virtual ~ICP() = default;

        /** Begins the ICP process for point clouds `source` and `target` with an initial
         * guess for the transform `guess`.*/
        void begin(const PointCloud& source, const PointCloud& target, const RBTransform& guess) {
            // Initial transform guess
            this->transform = guess;

            // Copy in point clouds
            this->a = source;
            this->b = target;

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
        [[nodiscard]] double calculate_cost() const {
            double sum_squares = 0.0;
            for (auto& match: matches) {
                sum_squares += match.cost;
            }
            return std::sqrt(sum_squares / a.cols());
        }

        /** The current transform. */
        [[nodiscard]] RBTransform current_transform() const {
            return transform;
        }

        /**
         * @brief Gets the current point matching computed by ICP.
         *
         * @return A reference to the matching. Invalidates if `begin` or `iterate` are called.
         */
        [[nodiscard]] const std::vector<Match>& get_matches() const {
            return matches;
        }
    };

    using ICP2 = ICP<Dimension::TwoD>;
    using ICP3 = ICP<Dimension::ThreeD>;

    template<>
    std::unordered_map<std::string, ICP2::MethodConstructor> ICP2::methods;

    template<>
    std::unordered_map<std::string, ICP3::MethodConstructor> ICP3::methods;
}

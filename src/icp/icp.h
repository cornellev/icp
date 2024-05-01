/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include "geo.h"

namespace icp {
    /**
     * Interface for iterative closest points.
     * You should interact with ICP instances through this API only.
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
     * 2. Call either `icp->converge(convergence_threshold)` or repeatedly
     * `icp->iterate()`.
     *
     * If these steps are not followed as described here, the behavior is
     * undefined.
     *
     * At any point in the process, you may query `icp->calculate_cost()` and
     * `icp->transform()`. Do note, however, that `icp->calculate_cost()` is not
     * a constant-time operation.
     */
    class ICP {
    protected:
        // TODO: make this more general to allow for point-to-line
        /** A point-to-point matching between `point` and `pair` at a distance
         * of `sqrt(sq_dist)`.  */
        struct Match {
            size_t point;
            size_t pair;
            double sq_dist;
        };

        /** The current point cloud transformation that is being optimized.
         */
        RBTransform transform;

        /** The centroid of the source point cloud. */
        Vector a_cm;

        /** The centroid of the destination point cloud. */
        Vector b_cm;

        /** The source point cloud relative to its centroid. */
        std::vector<Vector> a;

        /** The destination point cloud relative to its centroid. */
        std::vector<Vector> b;

        /** Keeps track of the previous cost to ensure that progress is being
         * made. @see ICP::current_cost. */
        double previous_cost;

        /** The RMS (root mean square) cost of the current transformation. */
        double current_cost;

        /** The pairing of each point in `a` to its closest in `b`. */
        std::vector<Match> matches;

        ICP();

        virtual void setup();

    public:
        /** The result of running `ICP::converge`. */
        struct ConvergenceReport {
            /** The least cost achieved. */
            double final_cost;

            /** The number of iterations performed, including the burn-in
             * period. */
            size_t iteration_count;
        };

        /** Configuration for ICP instances. */
        class Config {
            using Param = std::variant<int, double, std::string>;
            std::unordered_map<std::string, Param> params;

        public:
            /** Constructs an empty configuration. */
            Config() {}

            /** Associates `key` with an integer, double, or string `value`. */
            template<typename T>
            void set(std::string key, T value) {
                params[key] = value;
            }

            /** Retrieves the integer, double, or string value associated with
             * `key`. */
            template<typename T>
            T get(std::string key, T otherwise) const {
                if (params.find(key) == params.end()) {
                    return otherwise;
                } else {
                    return std::get<T>(params.at(key));
                }
            }
        };

        virtual ~ICP() = default;

        /** Begins the ICP process for point clouds `a` and `b` with an initial
         * guess for the transform `t`. */
        void begin(const std::vector<Vector>& a, const std::vector<Vector>& b,
            RBTransform t);

        /** Perform one iteration of ICP for the point clouds `a` and `b`
         * provided with ICP::begin. */
        virtual void iterate() = 0;

        /**
         * Computes the cost of the current transformation.
         *
         * \par Efficiency:
         * `O(a.size())` where `a` is the source point cloud.
         */
        double calculate_cost() const;

        /**
         * Perform ICP for the point clouds `a` and `b` provided with ICP::begin
         * until the cost is below `convergence_threshold` or until no progress
         * is being made. At least `burn_in` iterations will be performed. Start
         * with zero burn-in, and slowly increase if convergence requirements
         * are not met.
         *
         * @returns Information about the convergence.
         */
        ConvergenceReport converge(size_t burn_in,
            double convergence_threshold);

        /** The current transform. */
        const RBTransform& current_transform() const;

        /** Registers a new ICP method that can be created with `constructor`,
         * returning `false` if `name` has already been registered. */
        static bool register_method(std::string name,
            std::function<std::unique_ptr<ICP>(const Config&)> constructor);

        /** Returns a current list of the names of currently registered ICP
         * methods. */
        static const std::vector<std::string>& registered_methods();

        /**
         * Factory constructor for the ICP method `name` with configuration
         * `config`.
         *
         * @pre `name` is a valid registered method. See
         * ICP::is_registered_method.
         */
        static std::unique_ptr<ICP> from_method(std::string name,
            const Config& params = Config());

        /** Whether `name` is a registered ICP method. */
        static bool is_registered_method(std::string name);
    };

    struct Methods {
        std::vector<std::string> registered_method_names;
        std::vector<std::function<std::unique_ptr<ICP>(const ICP::Config&)>>
            registered_method_constructors;
    };
}

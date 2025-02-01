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
#include <variant>
#include <optional>

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
    class ICP {
    protected:
        // the dimension of icp(only 2 and 3 are legal)
        int dim;

        /** A matching between `point` and `pair` at (arbitrary) cost `cost`.  */
        struct Match {
            // need to be fix for xd?
            size_t point;
            size_t pair;
            double cost;
        };

        /** The current point cloud transformation that is being optimized. */
        RBTransform transform;

        /** The source point cloud. */
        std::vector<Vector> a;

        /** The destination point cloud. */
        std::vector<Vector> b;

        /** The pairing of each point in `a` to its closest in `b`. */
        std::vector<Match> matches;

        // Use matrix to represent the point clouds instead of set of vectors
        Eigen::MatrixXd A;  // a_matrix
        Eigen::MatrixXd B;  // b_matrix

        void convert_a_to_matrix() {
            A.resize(a.size(), dim);
            for (size_t i = 0; i < a.size(); ++i) {
                for (int j = 0; j < dim; ++j) {
                    A(i, j) = a[i][j];
                }
            }
        }

        void convert_b_to_matrix() {
            B.resize(b.size(), dim);
            for (size_t i = 0; i < b.size(); ++i) {
                for (int j = 0; j < dim; ++j) {
                    B(i, j) = b[i][j];
                }
            }
        }

        ICP();

        ICP(int dim): dim(dim), transform(RBTransform(dim)) {
            if (dim != 2 && dim != 3) {
                throw std::invalid_argument("Dimension must be 2 or 3");
            }
        }

        /**
         * @brief Per-method setup code.
         *
         * @post For implementers: must fill `matches` with match data for the initial point clouds.
         */
        virtual void setup() = 0;

    public:
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
         * guess for the transform `t`.*/
        void begin(const std::vector<Vector>& a, const std::vector<Vector>& b, RBTransform t);

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
        double calculate_cost() const;

        /** The current transform. */
        const RBTransform& current_transform() const;

        /** The matches. */
        const std::vector<Match>& get_matches() const;

        /** The dimensionality supported by this ICP instance. */
        int dimensionality() const;

        /** Registers methods built into `libcevicp`. Must be called before constructing ICP
         * instances for built-in methods. */
        static void register_builtin_methods();

        /** Registers a new ICP method that can be created with `constructor`,
         * returning `false` if `name` has already been registered. */
        static bool register_method(std::string name,
            std::function<std::unique_ptr<ICP>(const Config&)> constructor);

        /** Returns `true` if the ICP method `name` is registered. */
        static bool is_method_registered(std::string name);

        /** Returns a current list of the names of currently registered ICP
         * methods. */
        static const std::vector<std::string>& registered_methods();

        /**
         * Factory constructor for the ICP method `name` with configuration
         * `config`.
         *
         * @pre `name` is a valid registered method. See
         * ICP::registered_methods.
         */
        static std::optional<std::unique_ptr<ICP>> from_method(std::string name,
            const Config& params = Config());

    private:
        struct Methods {
            std::vector<std::string> registered_method_names;
            std::vector<std::function<std::unique_ptr<ICP>(const ICP::Config&)>>
                registered_method_constructors;
        };

        static Methods global;
        static bool builtins_registered;

        /** Registers methods built into `libcevicp`. Called automatically such that builtins will
         * always appear registered to users. */
        static void ensure_builtins_registered();

        /** Internal method registration that doesn't do any checks. */
        static void register_method_internal(std::string name,
            std::function<std::unique_ptr<ICP>(const Config&)> constructor);
    };
}
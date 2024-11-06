/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <numeric>

#include "icp/icp.h"

// methods for builtin registration
#include "icp/impl/vanilla.h"
#include "icp/impl/trimmed.h"

namespace icp {
    struct Methods {
        std::vector<std::string> registered_method_names;
        std::vector<std::function<std::unique_ptr<ICP>(const ICP::Config&)>>
            registered_method_constructors;
    };

    static Methods* global;

    ICP::ICP() {}

    void ICP::setup() {}

    void ICP::begin(const std::vector<Vector>& a, const std::vector<Vector>& b, RBTransform t) {
        // Initial transform guess
        this->transform = t;

        // Copy in point clouds
        this->a = a;
        this->b = b;

        // Cost is infinite initially
        previous_cost = std::numeric_limits<double>::infinity();
        current_cost = std::numeric_limits<double>::infinity();

        // Ensure arrays are the right size
        const size_t n = this->a.size();
        if (matches.size() < n) {
            matches.resize(n);
        }

        // Per-instance customization routine
        setup();
    }

    double ICP::calculate_cost() const {
        double sum_squares{};
        for (auto& match: matches) {
            sum_squares += match.sq_dist;
        }
        return std::sqrt(sum_squares / a.size());
    }

    ICP::ConvergenceReport ICP::converge(size_t burn_in, double convergence_threshold) {
        ConvergenceReport result{};

        // Repeat until convergence
        while (current_cost > convergence_threshold || current_cost == INFINITY
               || result.iteration_count < burn_in) {
            // Store previous iteration results
            previous_cost = current_cost;
            RBTransform previous_transform = transform;

            iterate();

            // If cost rose, revert to previous transformation/cost and
            // exit
            current_cost = calculate_cost();
            if (current_cost >= previous_cost && result.iteration_count > burn_in) {
                transform = previous_transform;
                current_cost = previous_cost;
                break;
            }

            result.iteration_count++;
        }

        result.final_cost = current_cost;

        return result;
    }

    const RBTransform& ICP::current_transform() const {
        return transform;
    }

    static void ensure_methods_exists() {
        if (!global) {
            global = new Methods();
        }
    }

    void ICP::register_builtin_methods() {
        register_method("vanilla",
            [](const ICP::Config& config) { return std::make_unique<Vanilla>(config); });
        register_method("trimmed",
            [](const ICP::Config& config) { return std::make_unique<Trimmed>(config); });
    }

    bool ICP::register_method(std::string name,
        std::function<std::unique_ptr<ICP>(const ICP::Config&)> constructor) {
        ensure_methods_exists();
        global->registered_method_constructors.push_back(constructor);
        global->registered_method_names.push_back(name);
        return true;
    }

    const std::vector<std::string>& ICP::registered_methods() {
        ensure_methods_exists();
        return global->registered_method_names;
    }

    std::optional<std::unique_ptr<ICP>> ICP::from_method(std::string name,
        const ICP::Config& config) {
        ensure_methods_exists();
        auto name_it = std::find(global->registered_method_names.begin(),
            global->registered_method_names.end(), name);

        if (name_it == global->registered_method_names.end()) {
            return {};
        }

        size_t index = name_it - global->registered_method_names.begin();

        return global->registered_method_constructors[index](config);
    }
}
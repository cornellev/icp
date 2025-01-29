/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <numeric>

#include "icp/icp.h"

// methods for builtin registration
#include "icp/impl/vanilla.h"
#include "icp/impl/trimmed.h"
#include "icp/impl/feature_aware.h"
#include "icp/impl/vanilla_3d.h"

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

        // Ensure arrays are the right size
        matches.resize(this->a.size());

        // Per-instance customization routine
        setup();
    }

    // this is also relying on previous match data right now...
    double ICP::calculate_cost() const {
        double sum_squares{};
        //double sum_squares = 0.0;
        for (auto& match: matches) {
            sum_squares += match.cost;
        }
        return std::sqrt(sum_squares / a.size());
    }

    const RBTransform& ICP::current_transform() const {
        return transform;
    }

    const std::vector<ICP::Match>& ICP::get_matches() const {
        return matches;
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
        register_method("feature_aware",
            [](const ICP::Config& config) { return std::make_unique<FeatureAware>(config); });
        register_method("vanilla_3d",
            [](const ICP::Config& config) { return std::make_unique<Vanilla_3d>(config); });
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

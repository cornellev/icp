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
        double sum_squares = 0.0;
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

    int ICP::dimensionality() const {
        return dim;
    }

    ICP::Methods ICP::global;
    bool ICP::builtins_registered = false;

    bool ICP::register_method(std::string name,
        std::function<std::unique_ptr<ICP>(const ICP::Config&)> constructor) {
        ensure_builtins_registered();

        if (is_method_registered(name)) {
            return false;
        }

        register_method_internal(name, constructor);
        return true;
    }

    bool ICP::is_method_registered(std::string name) {
        ensure_builtins_registered();

        return std::find(global.registered_method_names.begin(),
                   global.registered_method_names.end(), name)
               != global.registered_method_names.end();
    }

    const std::vector<std::string>& ICP::registered_methods() {
        ensure_builtins_registered();

        return global.registered_method_names;
    }

    std::optional<std::unique_ptr<ICP>> ICP::from_method(std::string name,
        const ICP::Config& config) {
        ensure_builtins_registered();

        auto name_it = std::find(global.registered_method_names.begin(),
            global.registered_method_names.end(), name);

        if (name_it == global.registered_method_names.end()) {
            return {};
        }

        size_t index = name_it - global.registered_method_names.begin();

        return global.registered_method_constructors[index](config);
    }

    void ICP::ensure_builtins_registered() {
        if (builtins_registered) {
            return;
        }

        register_method_internal("vanilla",
            [](const ICP::Config& config) { return std::make_unique<Vanilla>(config); });
        register_method_internal("trimmed",
            [](const ICP::Config& config) { return std::make_unique<Trimmed>(config); });
        register_method_internal("feature_aware",
            [](const ICP::Config& config) { return std::make_unique<FeatureAware>(config); });
        // register_method_internal("vanilla_3d",
        //     [](const ICP::Config& config) { return std::make_unique<Vanilla_3d>(config); });

        builtins_registered = true;
    }

    void ICP::register_method_internal(std::string name,
        std::function<std::unique_ptr<ICP>(const Config&)> constructor) {
        global.registered_method_constructors.push_back(constructor);
        global.registered_method_names.push_back(name);
    }
}

#include "icp/icp.h"
#include <unordered_map>

#include "icp/impl/vanilla.h"
#include "icp/impl/trimmed.h"
#include "icp/impl/feature_aware.h"

namespace icp {

    template<>
    void ICP2::ensure_builtins_registered() {
        methods["vanilla"] = [](const Config& config) { return std::make_unique<Vanilla>(config); };

        methods["trimmed"] = [](const Config& config) { return std::make_unique<Trimmed>(config); };

        methods["feature_aware"] = [](const Config& config) {
            return std::make_unique<FeatureAware>(config);
        };
    }

    // template<>
    // void ICP3::ensure_builtins_registered() {

    // }
}

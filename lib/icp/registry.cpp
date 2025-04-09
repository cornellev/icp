#include "icp/registry.h"
#include "icp/impl/vanilla.h"
#include "icp/impl/trimmed.h"
#include "icp/impl/feature_aware.h"
#include "icp/impl/vanilla_3d.h"

namespace icp {
    template<>
    void ICPRegistry<2>::use_builtins() {
        methods["vanilla"] = [](const Config& config) { return std::make_unique<Vanilla>(config); };
        methods["trimmed"] = [](const Config& config) { return std::make_unique<Trimmed>(config); };
        methods["feature_aware"] = [](const Config& config) {
            return std::make_unique<FeatureAware>(config);
        };
    }

    template<>
    void ICPRegistry<3>::use_builtins() {
        methods["vanilla"] = [](const Config& config) {
            return std::make_unique<Vanilla_3d>(config);
        };
    }
}

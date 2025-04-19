#include "icp/icp.h"
#include "icp/impl/vanilla.h"
#include "icp/impl/trimmed.h"
#include "icp/impl/feature_aware.h"
#include "icp/impl/vanilla_3d.h"
#include "icp/impl/trimmed_3d.h"

#define CONSTRUCT_CONFIG(T) [](const Config& config) { return std::make_unique<T>(config); }

namespace icp {
    template<>
    std::unordered_map<std::string, ICP2::MethodConstructor> ICP2::methods{
        {"vanilla", CONSTRUCT_CONFIG(Vanilla)},
        {"trimmed", CONSTRUCT_CONFIG(Trimmed)},
        {"feature_aware", CONSTRUCT_CONFIG(FeatureAware)},
    };

    template<>
    std::unordered_map<std::string, ICP3::MethodConstructor> ICP3::methods{
        {"vanilla", CONSTRUCT_CONFIG(Vanilla_3d)},
    };
}
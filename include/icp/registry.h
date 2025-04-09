/**
 * @copyright Copyright (c) 2025 Cornell Electric Vehicles
 *
 */

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include "icp/icp.h"
#include "icp/config.h"

namespace icp {
    template<const size_t Dim>
    class ICPRegistry {
    private:
        using MethodConstructor = std::function<std::unique_ptr<ICP<Dim>>(const Config&)>;
        std::unordered_map<std::string, MethodConstructor> methods;

    public:
        ICPRegistry() {
            use_builtins();
        }

        std::optional<std::unique_ptr<ICP<Dim>>> from_name(const std::string& name,
            const Config& config) {
            if (methods.find(name) == methods.end()) {
                return {};
            }

            return methods[name](config);
        }

        bool register_method(const std::string& name, MethodConstructor constructor) {
            if (is_method_registered(name)) {
                return false;
            }

            methods[name] = constructor;
            return true;
        }

        bool is_method_registered(const std::string& name) {
            return methods.find(name) != methods.end();
        }

        std::vector<std::string> registered_methods() {
            std::vector<std::string> keys;
            for (auto it = methods.begin(); it != methods.end(); ++it) {
                keys.push_back(it->first);
            }
            return keys;
        }

        void use_builtins();
    };

    using ICPRegistry2 = ICPRegistry<2>;
    using ICPRegistry3 = ICPRegistry<3>;

    template<>
    void ICPRegistry<2>::use_builtins();

    template<>
    void ICPRegistry<3>::use_builtins();
}
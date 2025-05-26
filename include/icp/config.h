#pragma once

#include <string>
#include <unordered_map>
#include <variant>

namespace icp {
    /** Configuration for ICP instances. */
    class Config {
        using Param = std::variant<int, double, std::string>;
        std::unordered_map<std::string, Param> params;

    public:
        /** Constructs an empty configuration. */
        Config() = default;

        /** Associates `key` with an integer, double, or string `value`. */
        template<typename T>
        void set(const std::string& key, T value) {
            params[key] = value;
        }

        /** Retrieves the integer, double, or string value associated with
         * `key`. */
        template<typename T>
        T get(const std::string& key, T otherwise) const {
            if (params.find(key) == params.end()) {
                return otherwise;
            } else {
                return std::get<T>(params.at(key));
            }
        }
    };
}

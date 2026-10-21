/**
 * @file
 * @brief Run vanilla 3D ICP on raw XYZI scans using the official zenoh-cpp API.
 * @copyright Copyright (C) 2026 Cornell Electric Vehicles. SPDX-License-Identifier: MIT
 *
 * @code{.sh}
 * cmake -S . -B build/zenoh -DCEV_ICP_BUILD_TOOLS=OFF -DCEV_ICP_BUILD_ZENOH=ON
 * cmake --build build/zenoh --target icp_zenoh
 * ./build/zenoh/icp_zenoh --listen tcp/127.0.0.1:7447
 * @endcode
 *
 * The first valid scan primes the reference. JSON results on rslidar/icp/registration
 * include per-scan rotation/translation, geometric RMSE, point counts, iterations,
 * termination, and registration duration. See icp_zenoh_support.h for the complete
 * wire/frame/metric contracts and book/zenoh.md for deployment examples.
 */
#include "icp_zenoh_support.h"

#include <csignal>
#include <iostream>
#include <limits>
#include <set>

namespace {
    volatile std::sig_atomic_t stopped = 0;

    void stop(int) {
        stopped = 1;
    }

    const char* help = R"(Usage: icp_zenoh [--option value]
Vanilla 3D scan matching over the official Zenoh C++ API.
  --listen ENDPOINT             Incoming endpoint (tcp/127.0.0.1:7447); "" disables it
  --connect ENDPOINT            Optional outgoing connection, in addition to listening
  --input-key KEY               Raw XYZI subscription (rslidar/points/raw)
  --output-key KEY              JSON results (rslidar/icp/registration)
  --frame-id ID                 Sensor frame label (rslidar); no coordinate conversion
  --max-points N                Declared record limit (1000000), before XYZ filtering
  --iterations N                Positive update cap (100)
  --time-limit-ms N             Cooperative time budget; disabled by default, zero allowed
  --rotation-tolerance RADIANS  Incremental rotation threshold (1e-6)
  --translation-tolerance UNITS Incremental translation threshold (1e-6)
  --help                       Print this help and exit
All numeric tolerances must be finite and nonnegative. Both transform tolerances
must be met. Limits take priority. Setup or one update can exceed a time budget.
Raw scan: little-endian i32 sec, u32 nanosec/count/stride, then f32 XYZI records.
Stride must be 16; payload length must be exact. Non-finite XYZ is discarded.
Results map CURRENT into REFERENCE: p_reference = R * p_current + t.
Rotation is quaternion XYZW; translation and RMSE use sensor length units.
The first valid scan produces no result. Latest-pending input/output drops are possible.
SIGINT/SIGTERM stop between processing phases. See book/zenoh.md for examples.
)";

    uint64_t count(const std::string& value) {
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
            throw std::runtime_error("expected an unsigned integer");
        }
        return std::stoull(value);
    }

    double tolerance(const std::string& value) {
        size_t consumed;
        const double result = std::stod(value, &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("invalid tolerance");
        }
        return result;
    }
}

int main(int argc, char** argv) {
    try {
        icp_zenoh::Settings settings;
        std::set<std::string> seen;
        for (int i = 1; i < argc; ++i) {
            const std::string name = argv[i];
            if (name == "--help") {
                std::cout << help;
                return 0;
            }
            if (++i >= argc || !seen.insert(name).second) {
                throw std::runtime_error("each option requires one value and cannot repeat");
            }
            const std::string value = argv[i];
            if (name == "--listen") {
                settings.listen = value;
            } else if (name == "--connect") {
                settings.connect = value;
            } else if (name == "--input-key") {
                settings.input_key = value;
            } else if (name == "--output-key") {
                settings.output_key = value;
            } else if (name == "--frame-id") {
                settings.frame_id = value;
            } else if (name == "--max-points" || name == "--iterations") {
                const auto number = count(value);
                if (number > std::numeric_limits<size_t>::max()) {
                    throw std::runtime_error("integer exceeds platform capacity");
                }
                (name == "--max-points" ? settings.max_points
                                        : settings.iterations) = static_cast<size_t>(number);
            } else if (name == "--time-limit-ms") {
                const auto number = count(value);
                if (number > static_cast<uint64_t>(std::chrono::milliseconds::max().count())
                    || std::chrono::duration<long double, std::milli>(
                           std::chrono::steady_clock::duration::max())
                               .count()
                           < number) {
                    throw std::runtime_error("time limit exceeds clock capacity");
                }
                settings.time_limit = std::chrono::milliseconds(number);
            } else if (name == "--rotation-tolerance") {
                settings.rotation_tolerance = tolerance(value);
            } else if (name == "--translation-tolerance") {
                settings.translation_tolerance = tolerance(value);
            } else {
                throw std::runtime_error("unknown option: " + name);
            }
        }
        icp_zenoh::validate_settings(settings);
        // Validate keys before opening transport so CLI errors cannot wait on a connection.
        const zenoh::KeyExpr input(settings.input_key);
        const zenoh::KeyExpr output(settings.output_key);
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        auto session = zenoh::Session::open(icp_zenoh::session_config(settings.listen,
            settings.connect));
        std::cerr << "icp_zenoh: " << settings.input_key << " -> " << settings.output_key
                  << ", frame " << settings.frame_id << '\n';
        icp_zenoh::run_node(std::move(session), settings, [] { return stopped != 0; });
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "icp_zenoh: " << error.what() << '\n';
        return 1;
    }
}

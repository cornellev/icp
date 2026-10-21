/**
 * @file
 * @brief Private application helpers for the optional C++ Zenoh node.
 * @copyright Copyright (C) 2026 Cornell Electric Vehicles. SPDX-License-Identifier: MIT
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <zenoh.hxx>
#include "icp/icp.h"

namespace icp_zenoh {
    using Json = nlohmann::json;

    /** Capture time from the payload; nanoseconds must be less than one billion. */
    struct Timestamp {
        int32_t sec;
        uint32_t nanosec;
    };

    /** One complete scan, with finite XYZ coordinates in the sensor's length units. */
    struct Frame {
        Timestamp stamp;
        icp::PointCloud3 points;
    };

    /** CLI defaults. Angles use radians; translation tolerances use input length units. */
    struct Settings {
        std::string listen = "tcp/127.0.0.1:7447";
        std::string connect;
        std::string input_key = "rslidar/points/raw";
        std::string output_key = "rslidar/icp/registration";
        std::string frame_id = "rslidar";
        size_t max_points = 1000000;
        size_t iterations = 100;
        std::optional<std::chrono::milliseconds> time_limit;
        double rotation_tolerance = 1e-6;
        double translation_tolerance = 1e-6;
    };

    /** Expected input/registration failure. what() is a stable snake-case error code. */
    struct Error : std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    /**
     * @brief Decode one little-endian raw XYZI scan, rejecting trailing bytes.
     * @details Header offsets: i32 seconds at 0, u32 nanoseconds at 4, u32 count at 8,
     * u32 stride at 12. Stride must be 16. Each record is four little-endian f32 values
     * X/Y/Z/intensity. Validate count and exact length before allocating point storage.
     * Non-finite XYZ records are discarded; intensity is ignored. No ROS/CDR decoding.
     */
    Frame decode_frame(const std::vector<uint8_t>& bytes, size_t max_points);

    /** Reject invalid limits and non-finite/negative tolerances before starting transport. */
    void validate_settings(const Settings& settings);

    /**
     * @brief Match current scans into the previous processed valid scan.
     * @details The first valid scan primes the reference and returns no message. Later
     * geometrically valid scans advance it even after alignment errors or limits. Decode
     * errors and invalid geometry leave it unchanged. No odometry is accumulated.
     *
     * Messages use schema_version 1 and current_to_reference direction. transform contains
     * translation [x,y,z] and a normalized quaternion_xyzw [x,y,z,w]. rmse is the geometric
     * nearest-neighbor RMSE at that exact pose, using all finite source points; retained_count
     * equals source_count for vanilla. initial_rmse uses identity. registration_ms includes
     * registration setup and residual evaluation but excludes payload decoding and publishing.
     * Limit messages contain candidate poses; error messages contain null poses/metrics.
     */
    class ScanMatcher {
    public:
        explicit ScanMatcher(Settings settings);
        std::optional<Json> process(Frame frame, const std::function<bool()>& cancelled);
        std::optional<Json> process_payload(const std::vector<uint8_t>& bytes,
            const std::function<bool()>& cancelled);

    private:
        Json message(std::optional<Timestamp> stamp) const;
        Settings settings_;
        std::optional<Frame> reference_;
    };

    /** Explicit peer endpoints with multicast discovery disabled; empty listen disables listening.
     */
    zenoh::Config session_config(const std::string& listen, const std::string& connect);

    /** Optional application-thread processing barriers for deterministic integration tests. */
    enum class WorkerEvent { Ready, Processed, Stopped };

    /**
     * @brief Own the session until shutdown, running all ICP/publishing in the caller's thread.
     * @details A capacity-one Zenoh ring keeps only the latest pending sample during overload.
     * References therefore follow processed scans, not necessarily consecutive sensor scans.
     * Output congestion control also permits drops. Track reference_stamp before chaining poses.
     * Cancellation/time limits are cooperative: setup or one processing phase can overrun.
     * Stop polling has a 10 ms idle delay; subscriber/publisher are destroyed before session close.
     */
    void run_node(zenoh::Session session, const Settings& settings,
        const std::function<bool()>& cancelled,
        const std::function<void(WorkerEvent)>& events = {});
}

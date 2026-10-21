// Copyright (C) 2026 Cornell Electric Vehicles. SPDX-License-Identifier: MIT
#include "icp_zenoh_support.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include <utility>
#include <Eigen/SVD>
#include "algo/kdtree.h"

namespace icp_zenoh {
    namespace {
        using Clock = std::chrono::steady_clock;

        uint32_t u32(const uint8_t* bytes) {
            return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16)
                   | (uint32_t(bytes[3]) << 24);
        }

        float f32(const uint8_t* bytes) {
            static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
            const auto bits = u32(bytes);
            float value;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        }

        Json timestamp(std::optional<Timestamp> stamp) {
            if (!stamp) {
                return nullptr;
            }
            return {{"sec", stamp->sec}, {"nanosec", stamp->nanosec}};
        }

        void validate_covariance(const Eigen::Matrix3d& covariance) {
            if (!covariance.allFinite()) {
                throw Error("numerical_failure");
            }
            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance);
            const auto values = svd.singularValues();
            if (!values.allFinite()) {
                throw Error("numerical_failure");
            }
            // Rank two admits planar scans while rejecting collinear or coincident geometry.
            if (values[0] <= 0 || values[1] <= values[0] * 1e-12) {
                throw Error("degenerate_geometry");
            }
        }

        void validate_cloud(const icp::PointCloud3& points) {
            if (points.cols() == 0) {
                throw Error("empty_cloud");
            }
            if (points.cols() < 3) {
                throw Error("insufficient_correspondences");
            }
            if (!points.allFinite()) {
                throw Error("non_finite_input");
            }
            const icp::PointCloud3 centered = points.colwise() - points.rowwise().mean();
            validate_covariance(centered * centered.transpose());
        }

        double residual(const icp::PointCloud3& source, const icp::PointCloud3& target,
            const icp::KdTree<Eigen::Vector3d>& tree, const icp::RBTransform3& transform) {
            if (!transform.matrix().allFinite()
                || !(transform.rotation().transpose() * transform.rotation())
                    .isApprox(Eigen::Matrix3d::Identity(), 1e-8)
                || std::abs(transform.rotation().determinant() - 1.0) > 1e-8) {
                throw Error("numerical_failure");
            }
            const icp::PointCloud3 transformed = transform * source;
            icp::PointCloud3 paired(3, source.cols());
            double sum = 0;
            for (Eigen::Index i = 0; i < source.cols(); ++i) {
                const Eigen::Vector3d point = transformed.col(i);
                const int index = tree.search(point);
                if (index < 0 || index >= target.cols()) {
                    throw Error("insufficient_correspondences");
                }
                paired.col(i) = target.col(index);
                // The legacy search distance/cost has different units; use coordinates directly.
                sum += (point - paired.col(i)).squaredNorm();
            }
            const icp::PointCloud3 a = transformed.colwise() - transformed.rowwise().mean();
            const icp::PointCloud3 b = paired.colwise() - paired.rowwise().mean();
            validate_covariance(a * b.transpose());
            const double rmse = std::sqrt(sum / static_cast<double>(source.cols()));
            if (!std::isfinite(rmse)) {
                throw Error("numerical_failure");
            }
            return rmse;
        }

        void align(const Frame& source, const Frame& target, const Settings& settings,
            const std::function<bool()>& cancelled, Json& report, Clock::time_point started) {
            if (cancelled()) {
                throw Error("cancelled");
            }
            std::vector<Eigen::Vector3d> target_points;
            target_points.reserve(target.points.cols());
            for (Eigen::Index i = 0; i < target.points.cols(); ++i) {
                target_points.emplace_back(target.points.col(i));
            }
            const icp::KdTree<Eigen::Vector3d> tree(target_points, 3);
            icp::RBTransform3 pose = icp::RBTransform3::Identity();
            const double initial_rmse = residual(source.points, target.points, tree, pose);
            auto algorithm = std::move(icp::ICP3::from_method("vanilla", icp::Config()).value());
            algorithm->begin(source.points, target.points, pose);
            size_t iterations = 0;
            double rmse = initial_rmse;
            const auto expired = [&] {
                return settings.time_limit && Clock::now() - started >= *settings.time_limit;
            };
            std::string termination;
            while (true) {
                if (cancelled()) {
                    throw Error("cancelled");
                }
                if (expired()) {
                    termination = "time_limit";
                    break;
                }
                if (iterations >= settings.iterations) {
                    termination = "maximum_iterations";
                    break;
                }
                const icp::RBTransform3 previous = pose;
                algorithm->iterate();
                ++iterations;
                pose = algorithm->current_transform();
                rmse = residual(source.points, target.points, tree, pose);
                if (cancelled()) {
                    throw Error("cancelled");
                }
                // Limits win over a tolerance reached on the same update.
                if (expired() || iterations >= settings.iterations) {
                    continue;
                }
                const Eigen::Matrix3d rotation_step = pose.rotation()
                                                      * previous.rotation().transpose();
                if (Eigen::AngleAxisd(rotation_step).angle() <= settings.rotation_tolerance
                    && (pose.translation() - previous.translation()).norm()
                           <= settings.translation_tolerance) {
                    termination = "transform_change";
                    break;
                }
            }
            Eigen::Quaterniond q(pose.rotation());
            q.normalize();
            report["status"] = termination == "transform_change" ? "converged" : "limit";
            report["termination"] = termination;
            report["iteration_count"] = iterations;
            report["retained_count"] = source.points.cols();
            report["initial_rmse"] = initial_rmse;
            report["rmse"] = rmse;
            report["transform"] = {{"translation", {pose.translation().x(), pose.translation().y(),
                                                       pose.translation().z()}},
                {"quaternion_xyzw", {q.x(), q.y(), q.z(), q.w()}}};
        }
    }

    void validate_settings(const Settings& settings) {
        if (settings.max_points == 0
            || settings.max_points > static_cast<size_t>(std::numeric_limits<int>::max())
            || settings.iterations == 0 || !std::isfinite(settings.rotation_tolerance)
            || settings.rotation_tolerance < 0 || !std::isfinite(settings.translation_tolerance)
            || settings.translation_tolerance < 0
            || (settings.time_limit
                && (settings.time_limit->count() < 0
                    || std::chrono::duration<long double, std::milli>(Clock::duration::max())
                               .count()
                           < settings.time_limit->count()))
            || settings.output_key.empty() || settings.output_key.find('*') != std::string::npos) {
            throw Error("invalid_config");
        }
    }

    Frame decode_frame(const std::vector<uint8_t>& bytes, size_t max_points) {
        if (bytes.size() < 16) {
            throw Error("truncated_header");
        }
        const uint32_t seconds = u32(bytes.data());
        const int32_t sec = seconds <= uint32_t(INT32_MAX) ? static_cast<int32_t>(seconds)
                                                           : -1 - static_cast<int32_t>(~seconds);
        const uint32_t nanosec = u32(bytes.data() + 4);
        if (nanosec >= 1000000000) {
            throw Error("invalid_timestamp");
        }
        const size_t count = u32(bytes.data() + 8);
        if (u32(bytes.data() + 12) != 16) {
            throw Error("invalid_stride");
        }
        if (count > max_points || count > static_cast<size_t>(std::numeric_limits<int>::max())) {
            throw Error("point_limit");
        }
        if (count > (std::numeric_limits<size_t>::max() - 16) / 16
            || bytes.size() != 16 + count * 16) {
            throw Error("invalid_length");
        }
        Frame frame{{sec, nanosec}, icp::PointCloud3(3, static_cast<Eigen::Index>(count))};
        Eigen::Index finite = 0;
        for (size_t i = 0; i < count; ++i) {
            const auto* record = bytes.data() + 16 + i * 16;
            Eigen::Vector3d point(f32(record), f32(record + 4), f32(record + 8));
            if (point.allFinite()) {
                frame.points.col(finite++) = point;
            }
        }
        frame.points.conservativeResize(3, finite);
        return frame;
    }

    ScanMatcher::ScanMatcher(Settings settings): settings_(std::move(settings)) {
        validate_settings(settings_);
    }

    Json ScanMatcher::message(std::optional<Timestamp> stamp) const {
        return {{"schema_version", 1}, {"current_stamp", timestamp(stamp)},
            {"reference_stamp",
                timestamp(reference_ ? std::optional(reference_->stamp) : std::nullopt)},
            {"frame_id", settings_.frame_id}, {"direction", "current_to_reference"},
            {"status", "error"}, {"transform", nullptr}, {"rmse", nullptr},
            {"initial_rmse", nullptr}, {"retained_count", nullptr}, {"source_count", nullptr},
            {"target_count", reference_ ? Json(reference_->points.cols()) : Json(nullptr)},
            {"iteration_count", 0}, {"termination", nullptr}, {"error_code", nullptr},
            {"registration_ms", 0.0}};
    }

    std::optional<Json> ScanMatcher::process(Frame frame, const std::function<bool()>& cancelled) {
        auto report = message(frame.stamp);
        report["source_count"] = frame.points.cols();
        const auto started = Clock::now();
        try {
            validate_cloud(frame.points);
        } catch (const Error& error) {
            report["error_code"] = error.what();
            return report;
        }
        if (!reference_) {
            reference_ = std::move(frame);
            return std::nullopt;
        }
        try {
            align(frame, *reference_, settings_, cancelled, report, started);
        } catch (const Error& error) {
            report["error_code"] = error.what();
        }
        report["registration_ms"] =
            std::chrono::duration<double, std::milli>(Clock::now() - started).count();
        reference_ = std::move(frame);
        return report;
    }

    std::optional<Json> ScanMatcher::process_payload(const std::vector<uint8_t>& bytes,
        const std::function<bool()>& cancelled) {
        try {
            return process(decode_frame(bytes, settings_.max_points), cancelled);
        } catch (const Error& error) {
            auto report = message(std::nullopt);
            report["error_code"] = error.what();
            return report;
        }
    }

    zenoh::Config session_config(const std::string& listen, const std::string& connect) {
        auto config = zenoh::Config::create_default();
        config.insert_json5("mode", "\"peer\"");
        config.insert_json5("scouting/multicast/enabled", "false");
        config.insert_json5("listen/endpoints",
            listen.empty() ? "[]" : Json::array({listen}).dump());
        config.insert_json5("connect/endpoints",
            connect.empty() ? "[]" : Json::array({connect}).dump());
        return config;
    }

    void run_node(zenoh::Session session, const Settings& settings,
        const std::function<bool()>& cancelled, const std::function<void(WorkerEvent)>& events) {
        ScanMatcher matcher(settings);
        {
            auto options = zenoh::Session::PublisherOptions::create_default();
            options.congestion_control = zenoh::CongestionControl::Z_CONGESTION_CONTROL_DROP;
            auto publisher = session.declare_publisher(zenoh::KeyExpr(settings.output_key),
                std::move(options));
            auto subscriber = session.declare_subscriber(zenoh::KeyExpr(settings.input_key),
                zenoh::channels::RingChannel(1));
            if (events) {
                events(WorkerEvent::Ready);
            }
            while (!cancelled()) {
                auto received = subscriber.handler().try_recv();
                if (auto* sample = std::get_if<zenoh::Sample>(&received)) {
                    auto report = matcher.process_payload(sample->get_payload().as_vector(),
                        cancelled);
                    if (report && !cancelled()) {
                        publisher.put(report->dump());
                    }
                    if (events) {
                        events(WorkerEvent::Processed);
                    }
                } else if (std::get<zenoh::channels::RecvError>(received)
                           == zenoh::channels::RecvError::Z_DISCONNECTED) {
                    throw std::runtime_error("Zenoh input channel disconnected");
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
        session.close();
        if (events) {
            events(WorkerEvent::Stopped);
        }
    }
}

// Copyright (C) 2026 Cornell Electric Vehicles. SPDX-License-Identifier: MIT
#include "icp_zenoh_support.h"

#include <atomic>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace icp_zenoh;
using namespace std::chrono_literals;

namespace {
    void check(bool condition, const char* message) {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    template<typename F>
    void error(F operation, const std::string& expected) {
        try {
            operation();
        } catch (const Error& e) {
            check(e.what() == expected, "wrong error code");
            return;
        }
        throw std::runtime_error("expected error: " + expected);
    }

    Frame cloud(int32_t sec, double shift = 0) {
        Frame frame{{sec, 123}, icp::PointCloud3(3, 100)};
        for (Eigen::Index i = 0; i < frame.points.cols(); ++i) {
            const double t = static_cast<double>(i);
            frame.points.col(i) << std::sin(t * 1.7) * 10 + shift, std::cos(t * 0.7) * 8,
                std::sin(t * 0.3) * 6;
        }
        return frame;
    }

    void write_u32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value) {
        for (size_t i = 0; i < 4; ++i) {
            bytes[offset + i] = static_cast<uint8_t>(value >> (i * 8));
        }
    }

    std::vector<uint8_t> encode(const Frame& frame) {
        std::vector<uint8_t> bytes(16 + static_cast<size_t>(frame.points.cols()) * 16);
        write_u32(bytes, 0, static_cast<uint32_t>(frame.stamp.sec));
        write_u32(bytes, 4, frame.stamp.nanosec);
        write_u32(bytes, 8, static_cast<uint32_t>(frame.points.cols()));
        write_u32(bytes, 12, 16);
        for (Eigen::Index i = 0; i < frame.points.cols(); ++i) {
            for (size_t axis = 0; axis < 3; ++axis) {
                const float value = static_cast<float>(frame.points(axis, i));
                uint32_t bits;
                std::memcpy(&bits, &value, 4);
                write_u32(bytes, 16 + static_cast<size_t>(i) * 16 + axis * 4, bits);
            }
        }
        return bytes;
    }

    icp::RBTransform3 pose(const Json& report) {
        const auto& q = report["transform"]["quaternion_xyzw"];
        const auto& t = report["transform"]["translation"];
        icp::RBTransform3 result = icp::RBTransform3::Identity();
        const Eigen::Quaterniond rotation(q[3], q[0], q[1], q[2]);
        check(std::abs(rotation.norm() - 1.0) < 1e-12, "quaternion is not normalized");
        result.linear() = rotation.toRotationMatrix();
        result.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
        return result;
    }

    double brute_rmse(const Frame& source, const Frame& target,
        const icp::RBTransform3& transform) {
        double sum = 0;
        for (Eigen::Index i = 0; i < source.points.cols(); ++i) {
            const Eigen::Vector3d point = transform * source.points.col(i);
            double best = std::numeric_limits<double>::infinity();
            for (Eigen::Index j = 0; j < target.points.cols(); ++j) {
                best = std::min(best, (point - target.points.col(j)).squaredNorm());
            }
            sum += best;
        }
        return std::sqrt(sum / static_cast<double>(source.points.cols()));
    }

    void unit_tests() {
        const auto never = [] { return false; };
        auto original = cloud(-2);
        const auto bytes = encode(original);
        const auto decoded = decode_frame(bytes, 100);
        check(decoded.stamp.sec == -2 && decoded.stamp.nanosec == 123, "timestamp roundtrip");
        check(decoded.points.cols() == 100, "point count");
        error([&] { decode_frame({1, 2}, 100); }, "truncated_header");
        error([&] { decode_frame(bytes, 99); }, "point_limit");
        auto bad = bytes;
        bad.pop_back();
        error([&] { decode_frame(bad, 100); }, "invalid_length");
        bad = bytes;
        bad.push_back(0);
        error([&] { decode_frame(bad, 100); }, "invalid_length");
        bad = bytes;
        write_u32(bad, 4, 1000000000);
        error([&] { decode_frame(bad, 100); }, "invalid_timestamp");
        bad = bytes;
        write_u32(bad, 12, 20);
        error([&] { decode_frame(bad, 100); }, "invalid_stride");
        bad = bytes;
        write_u32(bad, 8, UINT32_MAX);
        error([&] { decode_frame(bad, SIZE_MAX); }, "point_limit");
        bad = bytes;
        write_u32(bad, 16, 0x7fc00000);                // NaN X: discard record.
        write_u32(bad, 32, 0x7f800000);                // Infinite X: discard record.
        write_u32(bad, 16 + 2 * 16 + 12, 0x7fc00000);  // NaN intensity: keep record.
        check(decode_frame(bad, 100).points.cols() == 98, "XYZ/intensity filtering");

        Settings settings;
        settings.frame_id = "lidar\n\"quoted\"";
        ScanMatcher matcher(settings);
        check(!matcher.process(original, never), "first frame must prime reference");
        auto invalid = matcher.process_payload({0}, never).value();
        check(invalid["transform"].is_null() && invalid["current_stamp"].is_null(),
            "decode error pose");
        check(invalid["reference_stamp"]["sec"] == -2, "decode error changed reference");
        Frame empty{{1, 0}, icp::PointCloud3(3, 0)};
        check(matcher.process(empty, never).value()["error_code"] == "empty_cloud",
            "empty geometry");
        Frame small{{1, 0}, icp::PointCloud3::Zero(3, 1)};
        check(matcher.process(small, never).value()["error_code"] == "insufficient_correspondences",
            "small geometry");
        Frame line{{1, 0}, icp::PointCloud3::Zero(3, 3)};
        line.points.row(0) << 0, 1, 2;
        check(matcher.process(line, never).value()["error_code"] == "degenerate_geometry",
            "line geometry");
        auto nonfinite = cloud(1);
        nonfinite.points(0, 0) = std::numeric_limits<double>::quiet_NaN();
        check(matcher.process(nonfinite, never).value()["error_code"] == "non_finite_input",
            "finite validation");
        auto overflow = cloud(1);
        overflow.points *= 1e200;
        check(matcher.process(overflow, never).value()["error_code"] == "numerical_failure",
            "numeric overflow");

        auto report = matcher.process(cloud(2, 0.01), never).value();
        check(report["status"] == "converged", "translation did not converge");
        check(report["reference_stamp"]["sec"] == -2, "invalid geometry changed reference");
        check(report["current_stamp"]["sec"] == 2 && report["schema_version"] == 1,
            "schema/timestamps");
        check(report["direction"] == "current_to_reference", "direction");
        check(std::abs(pose(report).translation().x() + 0.01) < 1e-9, "translation direction");
        check(report["retained_count"] == 100 && report["source_count"] == 100
                  && report["target_count"] == 100,
            "point counts");
        check(Json::parse(report.dump())["frame_id"] == settings.frame_id, "JSON string escaping");
        check(report["registration_ms"].get<double>() >= 0, "duration");

        ScanMatcher exact(settings);
        exact.process(cloud(1), never);
        report = exact.process(cloud(2), never).value();
        check(report["status"] == "converged" && report["rmse"].get<double>() < 1e-12,
            "identity fit");
        Frame planar{{1, 0}, icp::PointCloud3(3, 4)};
        planar.points << 0, 2, 0, 2, 0, 0, 3, 3, 0, 0, 0, 0;
        ScanMatcher plane(settings);
        check(!plane.process(planar, never), "planar cloud rejected");
        check(plane.process(planar, never).value()["status"] == "converged", "planar alignment");

        auto source = cloud(1);
        auto target = cloud(2);
        icp::RBTransform3 expected = icp::RBTransform3::Identity();
        expected.linear() =
            Eigen::AngleAxisd(0.03, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
        expected.translation() = Eigen::Vector3d(0.1, -0.2, 0.05);
        target.points = expected * source.points;
        for (Eigen::Index i = 0; i < target.points.cols(); ++i) {
            target.points(0, i) += std::sin(static_cast<double>(i)) * 0.001;
        }
        ScanMatcher noisy(settings);
        noisy.process(target, never);
        report = noisy.process(source, never).value();
        const auto actual = pose(report);
        check((actual.translation() - expected.translation()).norm() < 0.001, "known translation");
        check((actual.rotation() - expected.rotation()).norm() < 0.001, "known rotation");
        check(std::abs(report["rmse"].get<double>() - brute_rmse(source, target, actual)) < 1e-12,
            "final RMSE does not describe reported pose");
        check(std::abs(report["initial_rmse"].get<double>()
                       - brute_rmse(source, target, icp::RBTransform3::Identity()))
                  < 1e-12,
            "initial RMSE");

        Settings cap;
        cap.iterations = 1;
        ScanMatcher capped(cap);
        capped.process(cloud(1), never);
        report = capped.process(cloud(2, 0.01), never).value();
        check(report["status"] == "limit" && report["termination"] == "maximum_iterations"
                  && report["iteration_count"] == 1 && !report["transform"].is_null(),
            "iteration cap");
        cap.time_limit = 0ms;
        ScanMatcher timed(cap);
        timed.process(cloud(1), never);
        report = timed.process(cloud(2), never).value();
        check(report["termination"] == "time_limit" && report["iteration_count"] == 0, "time cap");

        ScanMatcher recovery(settings);
        recovery.process(cloud(1), never);
        auto far = cloud(2, 10000);
        report = recovery.process(far, never).value();
        check(report["error_code"] == "degenerate_geometry" && report["transform"].is_null(),
            "underconstrained correspondence set");
        far.stamp.sec = 3;
        report = recovery.process(far, never).value();
        check(report["reference_stamp"]["sec"] == 2 && report["status"] == "converged",
            "failed valid alignment must advance reference");
        report = recovery.process(cloud(4), [] { return true; }).value();
        check(report["error_code"] == "cancelled" && report["transform"].is_null(), "cancellation");

        for (int setting = 0; setting < 5; ++setting) {
            Settings bad_settings;
            if (setting == 0) bad_settings.iterations = 0;
            if (setting == 1) bad_settings.max_points = 0;
            if (setting == 2) bad_settings.rotation_tolerance = -1;
            if (setting == 3)
                bad_settings.translation_tolerance = std::numeric_limits<double>::infinity();
            if (setting == 4) bad_settings.time_limit = -1ms;
            error([&] { ScanMatcher invalid_settings(bad_settings); }, "invalid_config");
        }
    }

    std::string endpoint() {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        check(fd >= 0, "socket failed");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            ::close(fd);
            throw std::runtime_error("localhost bind failed");
        }
        socklen_t size = sizeof(address);
        const int result = ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size);
        ::close(fd);
        check(result == 0, "getsockname failed");
        return "tcp/127.0.0.1:" + std::to_string(ntohs(address.sin_port));
    }

    template<typename F>
    void await(std::mutex& mutex, std::condition_variable& changed, F ready) {
        std::unique_lock lock(mutex);
        check(changed.wait_for(lock, 10s, ready), "timed out waiting for processing barrier");
    }

    void loopback() {
        const auto address = endpoint();
        auto server = zenoh::Session::open(session_config(address, ""));
        auto client = zenoh::Session::open(session_config("", address));
        std::mutex mutex;
        std::condition_variable changed;
        std::vector<Json> replies;
        Settings settings;
        auto subscriber = client.declare_subscriber(
            zenoh::KeyExpr(settings.output_key),
            [&](const zenoh::Sample& sample) {
                std::lock_guard lock(mutex);
                replies.push_back(Json::parse(sample.get_payload().as_string()));
                changed.notify_all();
            },
            zenoh::closures::none);
        auto publisher = client.declare_publisher(zenoh::KeyExpr(settings.input_key));
        auto matching = publisher.declare_matching_listener(
            [&](const zenoh::MatchingStatus&) { changed.notify_all(); }, zenoh::closures::none);
        size_t processed = 0;
        bool ready = false, stopped = false;
        std::atomic<bool> cancelled{false};
        auto worker = std::async(std::launch::async, [&] {
            run_node(
                std::move(server), settings, [&] { return cancelled.load(); },
                [&](WorkerEvent event) {
                    std::unique_lock lock(mutex);
                    if (event == WorkerEvent::Ready) ready = true;
                    if (event == WorkerEvent::Stopped) stopped = true;
                    if (event == WorkerEvent::Processed) {
                        ++processed;
                        changed.notify_all();
                    }
                    changed.notify_all();
                });
        });
        try {
            await(mutex, changed,
                [&] { return ready && publisher.get_matching_status().matching; });
            publisher.put(encode(cloud(1)));
            await(mutex, changed, [&] { return processed == 1; });
            {
                std::lock_guard lock(mutex);
                check(replies.empty(), "priming scan published a result");
            }
            publisher.put(encode(cloud(2, 0.01)));
            await(mutex, changed, [&] { return processed == 2 && replies.size() == 1; });
            {
                std::lock_guard lock(mutex);
                const auto& report = replies.back();
                check(report["status"] == "converged" && report["reference_stamp"]["sec"] == 1
                          && report["current_stamp"]["sec"] == 2,
                    "transport registration");
                check(std::abs(pose(report).translation().x() + 0.01) < 1e-5,
                    "transport pose direction");
            }
            publisher.put(std::vector<uint8_t>{0, 1});
            await(mutex, changed, [&] { return processed == 3 && replies.size() == 2; });
            {
                std::lock_guard lock(mutex);
                check(replies.back()["error_code"] == "truncated_header",
                    "transport decode failure");
            }
            publisher.put(encode(cloud(3, 0.02)));
            await(mutex, changed, [&] { return processed == 4 && replies.size() == 3; });
            {
                std::lock_guard lock(mutex);
                check(replies.back()["reference_stamp"]["sec"] == 2,
                    "transport error changed reference");
            }

            // Fill the actual Zenoh ring through its callback, waiting until every arrival
            // has completed before reading. This avoids racing independent subscribers.
            auto ring = zenoh::channels::RingChannel(1).into_cb_handler_pair<zenoh::Sample>();
            size_t arrivals = 0;
            {
                auto buffered = client.declare_subscriber(
                    zenoh::KeyExpr("test/ring"),
                    [&](const zenoh::Sample& sample) {
                        auto copy = sample.clone();
                        ::z_closure_sample_call(::z_loan(ring.first),
                            zenoh::interop::as_loaned_c_ptr(copy));
                        std::lock_guard lock(mutex);
                        ++arrivals;
                        changed.notify_all();
                    },
                    zenoh::closures::none);
                auto ring_publisher = client.declare_publisher(zenoh::KeyExpr("test/ring"));
                for (int i = 0; i < 100; ++i) ring_publisher.put(std::to_string(i));
                await(mutex, changed, [&] { return arrivals == 100; });
                auto last = ring.second.try_recv();
                check(std::get<zenoh::Sample>(last).get_payload().as_string() == "99",
                    "capacity-one ring did not retain latest sample");
                check(std::get<zenoh::channels::RecvError>(ring.second.try_recv())
                          == zenoh::channels::RecvError::Z_NODATA,
                    "ring retained older samples");
            }
            ::z_drop(::z_move(ring.first));
            cancelled = true;
            check(worker.wait_for(10s) == std::future_status::ready, "shutdown timeout");
            worker.get();
            check(stopped, "missing shutdown event");
        } catch (...) {
            cancelled = true;
            worker.wait();
            throw;
        }
    }
}

int main(int argc, char** argv) {
    try {
        check(argc == 2, "expected --unit or --loopback");
        if (std::string(argv[1]) == "--unit") {
            unit_tests();
        } else {
            check(std::string(argv[1]) == "--loopback", "unknown test mode");
            loopback();
        }
        std::cout << argv[1] << " passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

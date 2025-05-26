/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * SPDX-License-Identifier: MIT
 */
#include <cstddef>
#include <string>
#include "icp/geo.h"
#include "algo/kdtree.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include <iostream>
#include <random>
#include <cassert>
#include <Eigen/Dense>

extern "C" {
#include <simple_test/simple_test.h>
}

constexpr double TRANS_EPS = 0.5;  // Translation tolerance (units)
constexpr double RAD_EPS = 0.01;   // Rotation tolerance (radians)
                                   //
#define assert_translation_eps(expected, real, eps)                                                \
    do {                                                                                           \
        assert_true(std::abs((real).x() - (expected).x()) < (eps));                                \
        assert_true(std::abs((real).y() - (expected).y()) < (eps));                                \
    } while (0)

#define assert_rotation_eps(expected, real, eps)                                                   \
    do {                                                                                           \
        assert_true(std::abs((real).smallestAngle() - (expected).smallestAngle()) < (eps));        \
    } while (0)

#define assert_translation(expected, real) assert_translation_eps(expected, real, TRANS_EPS)

#define assert_rotation(expected, real) assert_rotation_eps(expected, real, RAD_EPS)

void test_kdtree() {
    using Vector = Eigen::Vector3d;
    std::vector<Vector> points;
    points.reserve(1000);

    std::mt19937 rng(123);
    std::uniform_real_distribution<double> dist(-100.0, 100.0);

    for (int i = 0; i < 1000; ++i) {
        points.emplace_back(dist(rng), dist(rng), dist(rng));
    }

    icp::KdTree<Vector> kdtree(points, 3);

    auto brute_force_nn = [&](const Vector& query) {
        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < points.size(); ++i) {
            double d = (points[i] - query).squaredNorm();
            if (d < best_dist) {
                best_dist = d;
                best_idx = static_cast<int>(i);
            }
        }
        return best_idx;
    };

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& query = points[i];
        int brute_idx = brute_force_nn(query);

        double kdtree_dist = 0;
        int kdtree_idx = kdtree.search(query, &kdtree_dist);

        if (kdtree_idx != brute_idx) {
            std::cerr << "Mismatch at point " << i << ": "
                      << "kdtree_idx = " << kdtree_idx << ", brute_idx = " << brute_idx << "\n";
            assert(false && "KDTree search mismatch brute force!");
        }
    }

    std::cout << "All KDTree tests passed!" << "\n";
}

void test_icp_generic(const std::string& method, const icp::Config& config) {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    std::unique_ptr<icp::ICP2> icp = icp::ICP2::from_method(method, config).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    // Test case 1: Single point translation
    {
        icp::PointCloud2 a(2, 1);
        a.col(0) << 0, 0;
        icp::PointCloud2 b(2, 1);
        b.col(0) << 100, 0;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(result.iteration_count <= 10);
        assert_translation(Eigen::Vector2d(100, 0), result.transform.translation());
        assert_rotation(Eigen::Rotation2Dd(0), Eigen::Rotation2Dd(result.transform.rotation()));
    }

    // Test case 2: Identity test
    {
        icp::PointCloud2 a(2, 2);
        a.col(0) << 0, 0;
        a.col(1) << 100, 100;
        icp::PointCloud2 b = a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_translation(Eigen::Vector2d(0, 0), result.transform.translation());
        assert_rotation(Eigen::Rotation2Dd(0), Eigen::Rotation2Dd(result.transform.rotation()));
    }

    // Test case 3a: Rotation at different angles
    for (int deg = 0; deg < 10; deg++) {
        double angle = (double)deg * M_PI / 180.0;
        Eigen::Rotation2Dd rotation(angle);

        icp::PointCloud2 a(2, 2);
        a.col(0) << -100, -100;
        a.col(1) << 100, 100;
        icp::PointCloud2 b = rotation * a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_translation(Eigen::Vector2d(0, 0), result.transform.translation());
        assert_rotation(rotation, Eigen::Rotation2Dd(result.transform.rotation()));
    }

    // Test case 3b: Rotation at different angles
    for (int deg = 0; deg < 10; deg++) {
        double angle = deg * M_PI / 180.0;
        Eigen::Rotation2Dd rotation(angle);

        icp::PointCloud2 a(2, 2);
        a.col(0) << 0, 0;
        a.col(1) << 1, 0;
        icp::PointCloud2 b = rotation * a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_translation(Eigen::Vector2d(0, 0), result.transform.translation());
        assert_rotation(rotation, Eigen::Rotation2Dd(result.transform.rotation()));
    }

    {
        // Test case 4: Pure translation along X-axis
        icp::PointCloud2 a(2, 2);
        a.col(0) << 0, 0;
        a.col(1) << 0, 100;
        icp::PointCloud2 b(2, 2);
        b.col(0) << 100, 0;
        b.col(1) << 100, 100;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_translation(Eigen::Vector2d(100, 0), result.transform.translation());
        assert_rotation(Eigen::Rotation2Dd(0), Eigen::Rotation2Dd(result.transform.rotation()));
    }

    {
        // Test case 5: Translation + rotation

        double angle = 45 * M_PI / 180.0;  // Rotate 45 degrees
        icp::Vector2 translation(5, 5);    // Translate by (5, 5)
        Eigen::Rotation2Dd rotation(angle);

        icp::RBTransform2 transform;
        transform.linear() = rotation.toRotationMatrix();
        transform.translation() = translation;

        icp::PointCloud2 a(2, 2);
        a.col(0) << 0, 0;
        a.col(1) << 100, 0;
        icp::PointCloud2 b = transform * a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_translation(translation, result.transform.translation());
        assert_rotation(rotation, Eigen::Rotation2Dd(result.transform.rotation()));
    }

    {
        // Test case 6: Add noise

        double angle = 30 * M_PI / 180.0;  // Rotate 30 degrees
        Eigen::Rotation2Dd rotation(angle);
        icp::Vector2 translation(20, 10);  // Translate by (20, 10)

        icp::RBTransform2 transform;
        transform.linear() = rotation.toRotationMatrix();
        transform.translation() = translation;

        std::default_random_engine generator;
        std::normal_distribution<double> noise_dist(0.0, 1.0);  // Mean 0, StdDev 1

        icp::PointCloud2 a(2, 4);
        a.col(0) << 0, 0;
        a.col(1) << 100, 0;
        a.col(2) << 50, 50;
        a.col(3) << 0, 50;

        icp::PointCloud2 b = transform * a;

        for (Eigen::Index i = 0; i < b.cols(); i++) {
            b.col(i) += Eigen::Vector2d(noise_dist(generator), noise_dist(generator));
        }

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());
        assert_translation_eps(translation, result.transform.translation(), TRANS_EPS * 3);
        assert_rotation_eps(rotation, Eigen::Rotation2Dd(result.transform.rotation()), RAD_EPS * 5);
    }
}

void test_main() {
    test_kdtree();

    test_icp_generic("vanilla", icp::Config());

    icp::Config trimmed_config;
    // fails with lower overlap rates on these super small examples
    trimmed_config.set("overlap_rate", 1.0);
    test_icp_generic("trimmed", trimmed_config);

    icp::Config feature_config;
    feature_config.set("overlap_rate", 1.0);
    feature_config.set("feature_weight", 0.7);
    feature_config.set("symmetric_neighbors", 1);
    test_icp_generic("feature_aware", feature_config);
}

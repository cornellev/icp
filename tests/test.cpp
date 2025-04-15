// Copyright (C) 2024 Ethan Uppal. All rights reserved.
#include <cstddef>
#include <string>
#include "icp/geo.h"
#include "algo/kdtree.h"  // Add this line to include the KdTree definition
#include "icp/icp.h"
#include "icp/driver.h"
#include <iostream>
#include <random>

extern "C" {
#include <simple_test/simple_test.h>
}

#define TRANS_EPS 0.5             // Translation tolerance in units
#define RAD_EPS ((double)(0.01))  // Rotation tolerance in radians
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

void test_kdtree(void) {
    std::vector<icp::Vector2> points;
    points.emplace_back(0, 0);
    points.emplace_back(1, 0);
    points.emplace_back(0, 1);
    points.emplace_back(1, 1);
    points.emplace_back(0.5, 0.5);

    std::cout << "Building KdTree with " << points.size() << " points" << std::endl;

    try {
        icp::KdTree<icp::Vector2> tree(points, 2);

        icp::Vector2 query(0.2, 0.2);
        float min_dist = 0;
        size_t nearest_idx = tree.find_nearest(query, &min_dist);

        std::cout << "Nearest point to (0.2, 0.2) is at index " << nearest_idx << " with distance "
                  << min_dist << std::endl;

        assert_true(nearest_idx == 0);

        query = icp::Vector2(0.6, 0.6);
        nearest_idx = tree.find_nearest(query, &min_dist);

        std::cout << "Nearest point to (0.6, 0.6) is at index " << nearest_idx << " with distance "
                  << min_dist << std::endl;

        assert_true(nearest_idx == 4);

        std::cout << "KdTree test passed" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "KdTree test failed: " << e.what() << std::endl;
        assert_true(false);
    }
}

void test_icp_generic(const std::string& method, const icp::Config& config) {
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
        std::normal_distribution<double> noise_dist(0.0, 1.0);

        icp::PointCloud2 a(2, 5);
        a.col(0) << 0, 0;
        a.col(1) << 100, 0;
        a.col(2) << 100, 100;
        a.col(3) << -20, 50;
        a.col(4) << 100, 120;
        icp::PointCloud2 b = transform * a;

        for (ptrdiff_t i = 0; i < b.cols(); i++) {
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

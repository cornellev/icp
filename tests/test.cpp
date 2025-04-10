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

#define BURN_IN 0                 // Minimum required iterations for the algorithm
#define TRANS_EPS 0.5             // Translation tolerance in units
#define RAD_EPS ((double)(0.01))  // Rotation tolerance in radians

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
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    // Test case 1: Single point translation
    {
        icp::PointCloud2 a{icp::Vector2(0, 0)};
        icp::PointCloud2 b{icp::Vector2(100, 0)};
        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(result.iteration_count <= BURN_IN + 10);
        assert_true(std::abs(result.transform.translation().x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(Eigen::Matrix2d::Identity()));
    }

    // Test case 2: Identity test
    {
        icp::PointCloud2 a{icp::Vector2(0, 0), icp::Vector2(100, 100)};
        icp::PointCloud2 b{icp::Vector2(0, 0), icp::Vector2(100, 100)};
        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(std::abs(result.transform.translation().x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(Eigen::Matrix2d::Identity()));
    }

    // Test case 3a: Rotation at different angles
    for (int deg = 0; deg < 10; deg++) {
        double angle = (double)deg * M_PI / 180.0;
        Eigen::Rotation2Dd rotation(angle);

        icp::PointCloud2 a{icp::Vector2(-100, -100), icp::Vector2(100, 100)};
        icp::PointCloud2 b = rotation * a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(std::abs(result.transform.translation().x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(rotation.toRotationMatrix()));
    }

    // Test case 3b: Rotation at different angles
    for (int deg = 0; deg < 10; deg++) {
        double angle = deg * M_PI / 180.0;
        Eigen::Rotation2Dd rotation(angle);

        icp::PointCloud2 a{icp::Vector2(0, 0), icp::Vector2(1, 0)};
        icp::PointCloud2 b = rotation * a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(result.transform.rotation().isApprox(rotation.toRotationMatrix()));
    }

    {
        // Test case 4: Pure translation along X-axis
        icp::PointCloud2 a{icp::Vector2(0, 0), icp::Vector2(0, 100)};
        icp::PointCloud2 b{icp::Vector2(100, 0), icp::Vector2(100, 100)};

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(std::abs(result.transform.translation().x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(Eigen::Matrix2d::Identity()));
    }

    {
        // Test case 5: Translation + rotation

        double angle = 45 * M_PI / 180.0;  // Rotate 45 degrees
        icp::Vector2 translation(5, 5);    // Translate by (5, 5)
        Eigen::Rotation2Dd rotation(angle);

        icp::RBTransform2 transform;
        transform.linear() = rotation.toRotationMatrix();
        transform.translation() = translation;

        icp::PointCloud2 a{icp::Vector2(0, 0), icp::Vector2(100, 0)};
        icp::PointCloud2 b = transform * a;

        auto result = driver.converge(a, b, icp::RBTransform2::Identity());

        assert_true(std::abs(result.transform.translation().x() - 5) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 5) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(transform.rotation()));
    }

    {
        // Test case 6: Add noise

        double angle = 30 * M_PI / 180.0;  // Rotate 30 degrees
        icp::Vector2 translation(20, 10);  // Translate by (20, 10)

        icp::RBTransform2 transform;
        transform.linear() = Eigen::Rotation2Dd(angle).toRotationMatrix();
        transform.translation() = translation;

        std::default_random_engine generator;
        std::normal_distribution<double> noise_dist(0.0, 1.0);

        icp::PointCloud2 a{icp::Vector2(0, 0), icp::Vector2(100, 0)};
        icp::PointCloud2 b = transform * a;

        for (ptrdiff_t i = 0; i < b.cols(); i++) {
            b.col(i) += Eigen::Vector2d(noise_dist(generator), noise_dist(generator));
        }

        auto result = driver.converge(a, b, icp::RBTransform2());

        assert_true(std::abs(result.transform.translation().x() - 20) <= TRANS_EPS + 1.0);
        assert_true(std::abs(result.transform.translation().y() - 10) <= TRANS_EPS + 1.0);
        // TODO: find a better way to test rotations
        assert_true(result.transform.rotation().isApprox(transform.rotation()));
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

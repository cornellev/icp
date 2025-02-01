// Copyright (C) 2024 Ethan Uppal. All rights reserved.
#include <string>
#include "icp/geo.h"
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

void test_kdtree(void) {}

void test_icp_generic(const std::string& method, const icp::ICP::Config& config) {
    // // Create ICP instance using the specified method and configuration
    // auto icp_result = icp::ICP::from_method(method, config);
    // assert_true(icp_result.has_value()); // Ensure ICP instance creation succeeded

    // std::unique_ptr<icp::ICP> icp = std::move(icp_result.value());

    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method, config).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    // Test case 1: Single point translation
    // {
    //     std::vector<icp::Vector> a = {icp::Vector(Eigen::Vector2d(0, 0))};
    //     std::vector<icp::Vector> b = {icp::Vector(Eigen::Vector2d(100, 0))};
    //     auto result = driver.converge(a, b, icp::RBTransform());

    //     // should not need more than 10 for such a trivial situation or else
    //     // there is a serious issue with the algorithm
    //     assert_true(result.iteration_count <= BURN_IN + 10);

    //     assert_true(std::abs(result.transform.translation.x() - 100) <= TRANS_EPS);
    //     assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    // }

    // Test case 1: Single point translation
    {
        std::vector<icp::Vector> a = {icp::Vector(Eigen::Vector2d(0, 0))};
        std::vector<icp::Vector> b = {icp::Vector(Eigen::Vector2d(100, 0))};
        auto result = driver.converge(a, b, icp::RBTransform(2));

        // Debug: Print transformation results
        std::cout << "[1]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[1]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[1]Result Iteration Count: " << result.iteration_count << std::endl;

        // Check iteration count
        assert_true(result.iteration_count <= BURN_IN + 10);

        // Check translation
        assert_true(std::abs(result.transform.translation.x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    // Test case 2: Identity test
    {
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(100, 100))};
        std::vector<icp::Vector> b = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(100, 100))};
        auto result = driver.converge(a, b, icp::RBTransform(2));

        std::cout << "[2]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[2]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[2]Result Iteration Count: " << result.iteration_count << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    // Test case 3: Rotation at different angles
    for (int deg = 0; deg < 20; deg++) {
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(-100, -100)), icp::Vector(Eigen::Vector2d(100, 100))};
        std::vector<icp::Vector> b = {};

        double angle = (double)deg * M_PI / 180.0;
        icp::Vector center = icp::get_centroid(a);
        icp::Matrix rotation_matrix{
            {std::cos(angle), -std::sin(angle)}, {std::sin(angle), std::cos(angle)}};

        for (const auto& point: a) {
            b.push_back(rotation_matrix * (point - center) + center);
        }

        std::cout << "testing angle: " << deg << '\n';

        auto result = driver.converge(a, b, icp::RBTransform(2));

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    {
        // Test case 4: Pure translation along X-axis
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(0, 100))};
        std::vector<icp::Vector> b = {
            icp::Vector(Eigen::Vector2d(100, 0)), icp::Vector(Eigen::Vector2d(100, 100))};

        auto result = driver.converge(a, b, icp::RBTransform(2));

        std::cout << "[3]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[3]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[3]Result Iteration Count: " << result.iteration_count << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    // need more test case that works for trimmed and feature aware(more points? since it filtered
    // out points)
    {
        // Translation + rotation
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(100, 0))};
        std::vector<icp::Vector> b;

        double angle = 45 * M_PI / 180.0;                  // Rotate 45 degrees
        icp::Vector translation(Eigen::Vector2d(50, 50));  // Translate by (50, 50)

        icp::Matrix rotation_matrix{
            {std::cos(angle), -std::sin(angle)}, {std::sin(angle), std::cos(angle)}};

        for (const auto& point: a) {
            b.push_back(rotation_matrix * point + translation);
        }

        auto result = driver.converge(a, b, icp::RBTransform(2));

        std::cout << "[4]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[4]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[4]Result Iteration Count: " << result.iteration_count << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 50) <= TRANS_EPS);
        // assert_true(std::abs(result.transform.rotation.angle() - angle) <= RAD_EPS);
    }

    {
        // Add noise
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(100, 0))};
        std::vector<icp::Vector> b;

        double angle = 30 * M_PI / 180.0;                  // Rotate 30 degrees
        icp::Vector translation(Eigen::Vector2d(20, 10));  // Translate by (20, 10)

        icp::Matrix rotation_matrix{
            {std::cos(angle), -std::sin(angle)}, {std::sin(angle), std::cos(angle)}};

        std::default_random_engine generator;
        std::normal_distribution<double> noise_dist(0.0,
            1.0);  // Noise with standard deviation of 1.0

        for (const auto& point: a) {
            Eigen::Vector2d noisy_point;
            noisy_point = rotation_matrix * point + translation;
            noisy_point.x() += noise_dist(generator);
            noisy_point.y() += noise_dist(generator);
            b.push_back(noisy_point);
        }

        auto result = driver.converge(a, b, icp::RBTransform());

        assert_true(std::abs(result.transform.translation.x() - 20) <= TRANS_EPS + 1.0);
        assert_true(std::abs(result.transform.translation.y() - 10) <= TRANS_EPS + 1.0);
    }
}

void test_main() {
    test_kdtree();

    test_icp_generic("vanilla", icp::ICP::Config());

    // icp::ICP::Config trimmed_config;
    // //fails with lower overlap rates on these super small examples
    // trimmed_config.set("overlap_rate", 1.0);
    // test_icp_generic("trimmed", trimmed_config);

    // icp::ICP::Config feature_config;
    // feature_config.set("overlap_rate", 1.0);
    // feature_config.set("feature_weight", 0.7);
    // feature_config.set("symmetric_neighbors", 1);
    // test_icp_generic("feature_aware", feature_config);
}

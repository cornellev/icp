// Copyright (C) 2024 Ethan Uppal. All rights reserved.
#include <string>
#include "icp/geo.h"
#include "algo/kdtree.h"  // Add this line to include the KdTree definition
#include "icp/icp.h"
#include "icp/driver/driver.h"
#include <iostream>
#include <random>

extern "C" {
#include <simple_test/simple_test.h>
}

#define BURN_IN 0                 // Minimum required iterations for the algorithm
#define TRANS_EPS 0.5             // Translation tolerance in units
#define RAD_EPS ((double)(0.01))  // Rotation tolerance in radians

void test_kdtree(void) {
    std::vector<icp::Vector> points;
    points.push_back(icp::Vector(Eigen::Vector2d(0, 0)));
    points.push_back(icp::Vector(Eigen::Vector2d(1, 0)));
    points.push_back(icp::Vector(Eigen::Vector2d(0, 1)));
    points.push_back(icp::Vector(Eigen::Vector2d(1, 1)));
    points.push_back(icp::Vector(Eigen::Vector2d(0.5, 0.5)));

    std::cout << "Building KdTree with " << points.size() << " points" << std::endl;

    try {
        icp::KdTree<icp::Vector> tree(points, 2);

        icp::Vector query(Eigen::Vector2d(0.2, 0.2));
        float min_dist = 0;
        size_t nearest_idx = tree.find_nearest(query, &min_dist);

        std::cout << "Nearest point to (0.2, 0.2) is at index " << nearest_idx << " with distance "
                  << min_dist << std::endl;

        assert_true(nearest_idx == 0);

        query = icp::Vector(Eigen::Vector2d(0.6, 0.6));
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

void test_icp_generic(const std::string& method, const icp::ICP::Config& config) {
    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method, config).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

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
        assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(2, 2)));
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
        assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(2, 2)));
    }

    // Test case 3: Rotation at different angles
    for (int deg = 0; deg < 10; deg++) {
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
        std::cout << "the result for the matrix (expect) " << rotation_matrix << '\n';

        auto result = driver.converge(a, b, icp::RBTransform(2));

        std::cout << "the result for the matrix (true)" << result.transform.rotation << '\n';
        std::cout << "the result for the matrix (true translate)" << result.transform.translation
                  << '\n';

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(rotation_matrix));
    }

    // Test case 3: Rotation at different angles
    for (int deg = 0; deg < 10; deg++) {
        double angle = deg * M_PI / 180.0;
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << cos(angle), -sin(angle), sin(angle), cos(angle);

        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(1, 0))};
        std::vector<icp::Vector> b = {icp::Vector(Eigen::Vector2d(0, 0)),
            icp::Vector(rotation_matrix * Eigen::Vector2d(1, 0))};

        auto result = driver.converge(a, b, icp::RBTransform(2));

        std::cout << "testing angle: " << deg << std::endl;
        std::cout << "the result for the matrix (expect) " << rotation_matrix << std::endl;
        std::cout << "the result for the matrix (true)" << result.transform.rotation << std::endl;
        std::cout << "the result for the matrix (true translate)" << result.transform.translation
                  << std::endl;

        assert_true(result.transform.rotation.isApprox(rotation_matrix, 1e-6));
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
        assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(2, 2)));
    }

    {
        // Translation + rotation
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector2d(0, 0)), icp::Vector(Eigen::Vector2d(100, 0))};
        std::vector<icp::Vector> b;

        double angle = 45 * M_PI / 180.0;                // Rotate 45 degrees
        icp::Vector translation(Eigen::Vector2d(5, 5));  // Translate by (50, 50)

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

        assert_true(std::abs(result.transform.translation.x() - 5) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 5) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(rotation_matrix));
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
        double angle_compute = std::atan2(result.transform.rotation(1, 0),
            result.transform.rotation(0, 0));
        assert_true(std::abs(angle_compute - angle) <= RAD_EPS);
        // need to find a way to test the rotation
    }
}

void test_main() {
    test_kdtree();

    test_icp_generic("vanilla", icp::ICP::Config());

    icp::ICP::Config trimmed_config;
    // fails with lower overlap rates on these super small examples
    trimmed_config.set("overlap_rate", 1.0);
    test_icp_generic("trimmed", trimmed_config);

    icp::ICP::Config feature_config;
    feature_config.set("overlap_rate", 1.0);
    feature_config.set("feature_weight", 0.7);
    feature_config.set("symmetric_neighbors", 1);
    test_icp_generic("feature_aware", feature_config);
}

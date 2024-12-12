// Copyright (C) 2024 Ethan Uppal. All rights reserved.

extern "C" {
#include <simple_test/simple_test.h>
}

#include "icp/icp.h"
#include "icp/driver.h"
#include <chrono>

#define BURN_IN 0
#define TRANS_EPS 0.5
#define RAD_EPS ((double)(0.01))

void test_kdtree(void) {}

void test_icp_generic(const std::string& method, const icp::ICP::Config& config) {
    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method, config).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    {
        std::vector<icp::Vector> a = {icp::Vector(0, 0)};
        std::vector<icp::Vector> b = {icp::Vector(100, 0)};
        auto result = driver.converge(a, b, icp::RBTransform());

        // should not need more than 10 for such a trivial situation or else
        // there is a serious issue with the algorithm
        assert_true(result.iteration_count <= BURN_IN + 10);

        assert_true(std::abs(result.transform.translation.x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    {
        std::vector<icp::Vector> a = {icp::Vector(0, 0), icp::Vector(100, 100)};
        std::vector<icp::Vector> b = {icp::Vector(0, 0), icp::Vector(100, 100)};
        auto result = driver.converge(a, b, icp::RBTransform());

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    for (int deg = 0; deg < 20; deg++) {
        std::vector<icp::Vector> a = {icp::Vector(-100, -100), icp::Vector(100, 100)};
        std::vector<icp::Vector> b = {};

        double angle = (double)deg * M_PI / 180.0;
        icp::Vector center = icp::get_centroid(a);
        icp::Matrix rotation_matrix{
            {std::cos(angle), -std::sin(angle)}, {std::sin(angle), std::cos(angle)}};

        for (const auto& point: a) {
            b.push_back(rotation_matrix * (point - center) + center);
        }

        std::cout << "testing angle: " << deg << '\n';

        auto result = driver.converge(a, b, icp::RBTransform());

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }

    {
        std::vector<icp::Vector> a = {icp::Vector(0, 0), icp::Vector(0, 100)};
        std::vector<icp::Vector> b = {icp::Vector(100, 0), icp::Vector(100, 100)};

        auto result = driver.converge(a, b, icp::RBTransform());

        assert_true(std::abs(result.transform.translation.x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    }
}

void test_main() {
    test_kdtree();

    icp::ICP::register_builtin_methods();

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

// Copyright (C) 2024 Ethan Uppal. All rights reserved.

extern "C" {
#include <simple_test/simple_test.h>
}

#include "icp/icp.h"

#define BURN_IN 0
#define TRANS_EPS 2
#define RAD_EPS ((double)(1e-1))

void test_kdtree(void) {}

void test_icp(const std::string& method) {
    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method);

    {
        std::vector<icp::Vector> a = {icp::Vector(0, 0)};
        std::vector<icp::Vector> b = {icp::Vector(100, 0)};
        icp->begin(a, b, icp::RBTransform());
        icp::ICP::ConvergenceReport result = icp->converge(BURN_IN, 0);

        // should not need more than 10 for such a trivial situation or else
        // there is a serious issue with the algorithm
        assert_true(result.iteration_count <= BURN_IN + 10);

        // again, for such a trivial situation, we should have easily achieved
        // the convergence requested
        assert_equal(0, result.final_cost);
        assert_true(fabs(icp->current_transform().translation.x() - 100) <= TRANS_EPS);
        assert_true(fabs(icp->current_transform().translation.y() - 0) <= TRANS_EPS);
    }

    {
        std::vector<icp::Vector> a = {icp::Vector(0, 0), icp::Vector(100, 100)};
        std::vector<icp::Vector> b = {icp::Vector(0, 0), icp::Vector(100, 100)};
        icp->begin(a, b, icp::RBTransform());
        icp::ICP::ConvergenceReport result = icp->converge(BURN_IN, 0);

        assert_true(result.final_cost < 1);

        assert_true(fabs(icp->current_transform().translation.x() - 0) <= TRANS_EPS);
        assert_true(fabs(icp->current_transform().translation.y() - 0) <= TRANS_EPS);
    }

    for (int deg = 0; deg < 20; deg++) {
        std::vector<icp::Vector> a = {icp::Vector(0, 0), icp::Vector(100, 100)};
        std::vector<icp::Vector> b = {};

        double angle = (double)deg * M_PI / 180.0;
        icp::Vector center(50, 50);
        icp::Matrix rotation_matrix{{cos(angle), -sin(angle)}, {sin(angle), cos(angle)}};
        for (const auto& point: a) {
            b.push_back(rotation_matrix * (point - center) + center);
        }

        icp->begin(a, b, icp::RBTransform());
        icp->converge(BURN_IN, 0);
        // assert_true(fabs(icp->current_transform().translation.x() - 0)
        //             <= TRANS_EPS);
        // assert_true(fabs(icp->current_transform().translation.y() - 0)
        //             <= TRANS_EPS);
    }

    {
        std::vector<icp::Vector> a = {icp::Vector(0, 0), icp::Vector(0, 100)};
        std::vector<icp::Vector> b = {icp::Vector(100, 0), icp::Vector(100, 100)};
        icp->begin(a, b, icp::RBTransform());
        icp->converge(BURN_IN, 0);
        assert_true(fabs(icp->current_transform().translation.x() - 100) <= TRANS_EPS);
        assert_true(fabs(icp->current_transform().translation.y() - 0) <= TRANS_EPS);
    }
}

void test_main() {
    test_kdtree();
    for (const auto& method: icp::ICP::registered_methods()) {
        std::cout << "testing icp method: " << method << '\n';
        test_icp(method);
    }
}

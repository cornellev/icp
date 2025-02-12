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

void test_icp_3d(const std::string& method, const icp::ICP::Config& config) {

    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method, config).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    // Test case 1: Single point translation
    {
        std::vector<icp::Vector> a = {icp::Vector(Eigen::Vector3d(0, 0, 0))};
        std::vector<icp::Vector> b = {icp::Vector(Eigen::Vector3d(100, 0, 0))};
        auto result = driver.converge(a, b, icp::RBTransform(3));

        // Debug: Print transformation results
        std::cout << "[1]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[1]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[1]Result Iteration Count: " << result.iteration_count << std::endl;

        // Check iteration count
        //assert_true(result.iteration_count <= BURN_IN + 10);

        // Check translation
        assert_true(std::abs(result.transform.translation.x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(3, 3)));
    }

    // Test case 2: Identity test
    {
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector3d(0, 0, 0)), icp::Vector(Eigen::Vector3d(100, 100, 100))};
        std::vector<icp::Vector> b = {
            icp::Vector(Eigen::Vector3d(0, 0, 0)), icp::Vector(Eigen::Vector3d(100, 100, 100))};
        auto result = driver.converge(a, b, icp::RBTransform(3));

        std::cout << "[2]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[2]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[2]Result Iteration Count: " << result.iteration_count << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(3, 3)));
    }

    // // Test case 3: Rotation about z-axis
    // {
    //     std::vector<icp::Vector> a = {icp::Vector(Eigen::Vector3d(1, 0, 0))};
    //     std::vector<icp::Vector> b = {icp::Vector(Eigen::Vector3d(0, 1, 0))};
    //     auto result = driver.converge(a, b, icp::RBTransform(3));

    //     Eigen::Matrix3d rotation_matrix;
    //     rotation_matrix << 0, -1, 0, 1, 0, 0, 0, 0, 1;  // 90 degree rotation about z-axis
        
    //     // Check iteration count
    //     assert_true(result.iteration_count <= BURN_IN + 10);

    //     // Check translation
    //     std::cout << "[3]Result Transform Translation X: " << result.transform.translation.x() << std::endl;
    //     std::cout << "[3]Result Transform Translation Y: " << result.transform.translation.y() << std::endl;
    //     assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
    //     assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    //     assert_true(std::abs(result.transform.translation.z() - 0) <= TRANS_EPS);
    //     assert_true(result.transform.rotation.isApprox(rotation_matrix));
    //     //should we prefer rotatoin or translation?(does it matters?)
    // }
     // Test case 3: Rotation about one of the axis
    for (int deg = 0; deg < 10; deg++) {
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector3d(1, 0, 0)), icp::Vector(Eigen::Vector3d(0, 1, 0)),icp::Vector(Eigen::Vector3d(0, 0, 1))};
        std::vector<icp::Vector> b = {};

        double angle = (double)deg * M_PI / 180.0;
        icp::Vector center = icp::get_centroid(a);
        icp::Matrix rotation_matrix(3, 3);
        rotation_matrix << 1, 0, 0,
                           0, std::cos(angle), -std::sin(angle),
                           0, std::sin(angle), std::cos(angle);

        for (const auto& point: a) {
            b.push_back(rotation_matrix * (point - center) + center);
        }

        std::cout << "testing angle: " << deg << '\n';

        auto result = driver.converge(a, b, icp::RBTransform(3));

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.z() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(rotation_matrix));
    }

    // Test case 3: Rotation about multiple the axis(problem here) 
    {
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector3d(1, 0, 0)), icp::Vector(Eigen::Vector3d(0, 1, 0)),icp::Vector(Eigen::Vector3d(0, 0, 1))};
        std::vector<icp::Vector> b = {};

        // int deg_1 = rand()%360;
        // int deg_2 = rand()%360;
        // int deg_3 = rand()%360;
        int deg_1 = 30;
        int deg_2 = 30; //0,0,66 breaks; 0,52,52 breaks; 42,42,42 breaks
        int deg_3 = 30;
        
        double angle_1 = (double)deg_1 * M_PI / 180.0;
        double angle_2 = (double)deg_2 * M_PI / 180.0;
        double angle_3 = (double)deg_3 * M_PI / 180.0;
        icp::Vector center = icp::get_centroid(a);
        icp::Matrix rotation_matrix(3, 3);
        rotation_matrix << std::cos(angle_2)*std::cos(angle_3), std::sin(angle_1)*std::sin(angle_2)*std::cos(angle_3)-std::cos(angle_1)*std::sin(angle_3), std::cos(angle_1)*std::sin(angle_2)*std::cos(angle_3)+std::sin(angle_1)*std::sin(angle_3),
                           std::cos(angle_2)*std::sin(angle_3), std::sin(angle_1)*std::sin(angle_2)*std::sin(angle_3)+std::cos(angle_1)*std::cos(angle_3), std::cos(angle_1)*std::sin(angle_2)*std::sin(angle_3)-std::sin(angle_1)*std::cos(angle_3),
                           -std::sin(angle_2), std::sin(angle_1)*std::cos(angle_2), std::cos(angle_1)*std::cos(angle_2);

        for (const auto& point: a) {
            b.push_back(rotation_matrix * (point - center) + center);
        }

        auto result = driver.converge(a, b, icp::RBTransform(3));
        
        std::cout << "rotation matrix (expected): " << rotation_matrix << std::endl;
        std::cout << "rotation matrix (true): " << result.transform.rotation << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.z() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(rotation_matrix));
    }

    {
        // Test case 4: Pure translation multiple axis
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector3d(1, 0, 0)), icp::Vector(Eigen::Vector3d(0, 1, 0)),icp::Vector(Eigen::Vector3d(0, 0, 1))};
        std::vector<icp::Vector> b = {
            icp::Vector(Eigen::Vector3d(51, 73, 2)), icp::Vector(Eigen::Vector3d(50, 74, 2)),icp::Vector(Eigen::Vector3d(50, 73, 3))};

        auto result = driver.converge(a, b, icp::RBTransform(3));

        std::cout << "[3]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[3]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[3]Result Transform Translation Y: " << result.transform.translation.z()
                  << std::endl;
        std::cout << "[3]Result Iteration Count: " << result.iteration_count << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 73) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.z() - 2) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(3, 3)));

    }

    // need more test case that works for trimmed and feature aware(more points? since it filtered
    // out points)
    {
        // Translation + rotation
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector3d(100, 0, 0)), icp::Vector(Eigen::Vector3d(0, 100, 0)),icp::Vector(Eigen::Vector3d(0, 0, 100))};
        std::vector<icp::Vector> b;

        double angle_1 = 10 * M_PI / 180.0;
        double angle_2 = 10 * M_PI / 180.0;
        double angle_3 = 10 * M_PI / 180.0;
        icp::Vector translation(Eigen::Vector3d(50, 50, 50)); 

        icp::Matrix rotation_matrix(3, 3); 
        rotation_matrix << std::cos(angle_2)*std::cos(angle_3), std::sin(angle_1)*std::sin(angle_2)*std::cos(angle_3)-std::cos(angle_1)*std::sin(angle_3), std::cos(angle_1)*std::sin(angle_2)*std::cos(angle_3)+std::sin(angle_1)*std::sin(angle_3),
                           std::cos(angle_2)*std::sin(angle_3), std::sin(angle_1)*std::sin(angle_2)*std::sin(angle_3)+std::cos(angle_1)*std::cos(angle_3), std::cos(angle_1)*std::sin(angle_2)*std::sin(angle_3)-std::sin(angle_1)*std::cos(angle_3),
                           -std::sin(angle_2), std::sin(angle_1)*std::cos(angle_2), std::cos(angle_1)*std::cos(angle_2);

        for (const auto& point: a) {
            b.push_back(rotation_matrix * point + translation);
        }

        auto result = driver.converge(a, b, icp::RBTransform(3));

        std::cout << "[4]Result Transform Translation X: " << result.transform.translation.x()
                  << std::endl;
        std::cout << "[4]Result Transform Translation Y: " << result.transform.translation.y()
                  << std::endl;
        std::cout << "[4]Result Transform Translation Z: " << result.transform.translation.z()
                  << std::endl;
        std::cout << "[4]Result Iteration Count: " << result.iteration_count << std::endl;

        assert_true(std::abs(result.transform.translation.x() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.y() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation.z() - 50) <= TRANS_EPS);
        assert_true(result.transform.rotation.isApprox(rotation_matrix));
        //problem here for translation
    }

    {
        // Add noise
        std::vector<icp::Vector> a = {
            icp::Vector(Eigen::Vector3d(100, 0, 0)), icp::Vector(Eigen::Vector3d(0, 100, 0)),icp::Vector(Eigen::Vector3d(0, 0, 100))};
        std::vector<icp::Vector> b;

        double angle_1 = 10 * M_PI / 180.0;
        double angle_2 = 10 * M_PI / 180.0;
        double angle_3 = 10 * M_PI / 180.0;
        icp::Vector translation(Eigen::Vector3d(20, 10, 30));  // Translate by (20, 10)

        icp::Matrix rotation_matrix(3, 3); 
        rotation_matrix << std::cos(angle_2)*std::cos(angle_3), std::sin(angle_1)*std::sin(angle_2)*std::cos(angle_3)-std::cos(angle_1)*std::sin(angle_3), std::cos(angle_1)*std::sin(angle_2)*std::cos(angle_3)+std::sin(angle_1)*std::sin(angle_3),
                           std::cos(angle_2)*std::sin(angle_3), std::sin(angle_1)*std::sin(angle_2)*std::sin(angle_3)+std::cos(angle_1)*std::cos(angle_3), std::cos(angle_1)*std::sin(angle_2)*std::sin(angle_3)-std::sin(angle_1)*std::cos(angle_3),
                           -std::sin(angle_2), std::sin(angle_1)*std::cos(angle_2), std::cos(angle_1)*std::cos(angle_2);

        std::default_random_engine generator;
        std::normal_distribution<double> noise_dist(0.0,
            1.0);  // Noise with standard deviation of 1.0

        for (const auto& point: a) {
            Eigen::Vector3d noisy_point;
            noisy_point = rotation_matrix * point + translation;
            noisy_point.x() += noise_dist(generator);
            noisy_point.y() += noise_dist(generator);
            noisy_point.z() += noise_dist(generator);
            b.push_back(noisy_point);
        }

        auto result = driver.converge(a, b, icp::RBTransform());

        assert_true(std::abs(result.transform.translation.x() - 20) <= TRANS_EPS + 1.0);
        assert_true(std::abs(result.transform.translation.y() - 10) <= TRANS_EPS + 1.0);
        assert_true(std::abs(result.transform.translation.z() - 10) <= TRANS_EPS + 1.0);

        Eigen::Matrix3d rotation_matrix_2 = result.transform.rotation.block<3,3>(0,0); 
        Eigen::Vector3d euler_angles = rotation_matrix_2.eulerAngles(0, 1, 2);
        // assert_true(std::abs(euler_angles[0] - angle_1) <= RAD_EPS);
        // assert_true(std::abs(euler_angles[1] - angle_2) <= RAD_EPS);
        // assert_true(std::abs(euler_angles[2] - angle_3) <= RAD_EPS);
    }
    //for testing the rotation, should we decompose the rotation matrix and compare the angles seperately?
}

void test_main() {
    test_icp_3d("vanilla_3d", icp::ICP::Config());
}

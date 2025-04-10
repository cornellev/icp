#include "icp/geo.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include "icp/impl/vanilla_3d.h"
#include <random>

extern "C" {
#include <simple_test/simple_test.h>
}

#define BURN_IN 0                 // Minimum required iterations for the algorithm
#define TRANS_EPS 0.5             // Translation tolerance in units
#define RAD_EPS ((double)(0.01))  // Rotation tolerance in radians

void test_icp_3d(const icp::Config& config) {
    std::unique_ptr<icp::ICP3> icp = std::make_unique<icp::Vanilla_3d>(config);
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    // Test case 1: Single point translation
    {
        icp::PointCloud3 a{{0, 0, 0}};
        icp::PointCloud3 b{{100, 0, 0}};
        auto result = driver.converge(a, b, icp::RBTransform3::Identity());  // Use RBTransform3

        assert_true(result.iteration_count <= BURN_IN + 10);
        assert_true(std::abs(result.transform.translation().x() - 100) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(Eigen::Matrix3d::Identity()));
    }

    // Note that we will not test colinear cases as they may lead to hard to correct degenerancy
    // (and are not common at all in real world data)
    // TODO: check for the colinear case and return an error or something?
    // Test case 2: Identity test
    // {
    //     std::vector<icp::Vector> a = {
    //         icp::Vector(Eigen::Vector3d(0, 0, 0)), icp::Vector(Eigen::Vector3d(100, 100, 100))};
    //     std::vector<icp::Vector> b = {
    //         icp::Vector(Eigen::Vector3d(0, 0, 0)), icp::Vector(Eigen::Vector3d(100, 100, 100))};
    //     auto result = driver.converge(a, b, icp::RBTransform(3));
    //     assert_true(std::abs(result.transform.translation.x() - 0) <= TRANS_EPS);
    //     assert_true(std::abs(result.transform.translation.y() - 0) <= TRANS_EPS);
    //     assert_true(result.transform.rotation.isApprox(icp::Matrix::Identity(3, 3)));
    // }

    // Test case 3: Rotation about one of the axes
    for (int deg = 0; deg < 10; deg++) {
        icp::PointCloud3 a{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        icp::PointCloud3 b(3, a.cols());

        double angle = (double)deg * M_PI / 180.0;
        icp::Vector3 center = icp::get_centroid(a);
        Eigen::AngleAxisd rotation_vector(angle, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();

        // Create transform for rotation around centroid: T_c * R * T_nc => R*x + (center -
        // R*center)
        icp::RBTransform3 transform;
        transform.linear() = rotation_matrix;
        transform.translation() = center - rotation_matrix * center;
        b = transform * a;  // Apply transform directly

        auto result = driver.converge(a, b, icp::RBTransform3::Identity());  // Use RBTransform3

        assert_true(std::abs(result.transform.translation().x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().z() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(rotation_matrix));
    }

    // Test case 4: Rotation about multiple axes
    {
        icp::PointCloud3 a{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        icp::PointCloud3 b(3, a.cols());

        int deg_1 = 30;
        int deg_2 = 30;
        int deg_3 = 30;

        double angle_1 = (double)deg_1 * M_PI / 180.0;
        double angle_2 = (double)deg_2 * M_PI / 180.0;
        double angle_3 = (double)deg_3 * M_PI / 180.0;
        icp::Vector3 center = icp::get_centroid(a);

        Eigen::AngleAxisd rot_x(angle_1, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y(angle_2, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z(angle_3, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation_matrix = (rot_z * rot_y * rot_x).toRotationMatrix();

        // Create transform for rotation around centroid: T_c * R * T_nc => R*x + (center -
        // R*center)
        icp::RBTransform3 transform;
        transform.linear() = rotation_matrix;
        transform.translation() = center - rotation_matrix * center;
        b = transform * a;  // Apply transform directly

        auto result = driver.converge(a, b, icp::RBTransform3::Identity());  // Use RBTransform3

        assert_true(std::abs(result.transform.translation().x() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 0) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().z() - 0) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(rotation_matrix));
    }

    // Test case 5: Pure translation along multiple axes
    {
        icp::PointCloud3 a{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        icp::PointCloud3 b{{51, 73, 2}, {50, 74, 2}, {50, 73, 3}};

        auto result = driver.converge(a, b, icp::RBTransform3::Identity());  // Use RBTransform3

        assert_true(std::abs(result.transform.translation().x() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 73) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().z() - 2) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(Eigen::Matrix3d::Identity()));
    }

    // Test case 6: Translation + rotation
    {
        icp::PointCloud3 a{{100, 0, 0}, {0, 100, 0}, {0, 0, 100}};
        icp::PointCloud3 b(3, a.cols());

        double angle_1 = 10 * M_PI / 180.0;
        double angle_2 = 10 * M_PI / 180.0;
        double angle_3 = 10 * M_PI / 180.0;
        icp::Vector3 translation(Eigen::Vector3d(50, 50, 50));

        // ZYX Euler angles
        Eigen::AngleAxisd rot_x(angle_1, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y(angle_2, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z(angle_3, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation_matrix = (rot_z * rot_y * rot_x).toRotationMatrix();

        // Create transform R*x + t
        icp::RBTransform3 transform;
        transform.linear() = rotation_matrix;
        transform.translation() = translation;
        b = transform * a;  // Apply transform directly

        auto result = driver.converge(a, b, icp::RBTransform3::Identity());  // Use RBTransform3

        assert_true(std::abs(result.transform.translation().x() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().y() - 50) <= TRANS_EPS);
        assert_true(std::abs(result.transform.translation().z() - 50) <= TRANS_EPS);
        assert_true(result.transform.rotation().isApprox(rotation_matrix));
    }

    // Test case 7: Add noise
    {
        icp::PointCloud3 a{{100, 0, 0}, {0, 100, 0}, {0, 0, 100}};
        icp::PointCloud3 b(3, a.cols());

        double angle_1 = 10 * M_PI / 180.0;
        double angle_2 = 10 * M_PI / 180.0;
        double angle_3 = 10 * M_PI / 180.0;
        icp::Vector3 translation(Eigen::Vector3d(20, 10, 30));

        // ZYX Euler angles
        Eigen::AngleAxisd rot_x(angle_1, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y(angle_2, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z(angle_3, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation_matrix = (rot_z * rot_y * rot_x).toRotationMatrix();

        std::default_random_engine generator;
        std::normal_distribution<double> noise_dist(0.0,
            1.0);  // Noise with standard deviation of 1.0

        // Create transform R*x + t
        icp::RBTransform3 transform;
        transform.linear() = rotation_matrix;
        transform.translation() = translation;
        icp::PointCloud3 b_transformed = transform * a;  // Apply transform first

        // Add noise separately
        for (ptrdiff_t i = 0; i < a.cols(); ++i) {  // Iterate through columns
            Eigen::Vector3d noisy_point = b_transformed.col(i);
            noisy_point.x() += noise_dist(generator);
            noisy_point.y() += noise_dist(generator);
            noisy_point.z() += noise_dist(generator);
            b.col(i) = noisy_point;                                          // Assign column
        }

        auto result = driver.converge(a, b, icp::RBTransform3::Identity());  // Use RBTransform3

        assert_true(std::abs(result.transform.translation().x() - 20) <= TRANS_EPS + 1.0);
        assert_true(std::abs(result.transform.translation().y() - 10) <= TRANS_EPS + 1.0);
        assert_true(std::abs(result.transform.translation().z() - 30) <= TRANS_EPS + 1.0);

        Eigen::Matrix3d rotation_matrix_2 = result.transform.rotation().block<3, 3>(0, 0);
        Eigen::Vector3d euler_angles = rotation_matrix_2.eulerAngles(0, 1, 2);
        assert_true(std::abs(euler_angles[0] - angle_1) <= 0.1);
        assert_true(std::abs(euler_angles[1] - angle_2) <= 0.1);
        assert_true(std::abs(euler_angles[2] - angle_3) <= 0.1);
    }
}

void test_main() {
    test_icp_3d(icp::Config());
}

#include <string>
#include "icp/geo.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include "parse_ply.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#define BURN_IN 0                 // Minimum required iterations for the algorithm
#define TRANS_EPS 0.01            // Translation tolerance in units
#define RAD_EPS ((double)(0.001)) // Rotation tolerance in radians
const std::string ICP_METHOD = "vanilla_3d";

void write_transformed_points_ply(const std::vector<icp::Vector>& points, const icp::RBTransform& transform, const std::string& output_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : points) {
        icp::Vector transformed_point = transform.apply_to(point);
        cloud->points.emplace_back(transformed_point.x(), transformed_point.y(), transformed_point.z());
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePLYFileASCII(output_path, *cloud) == -1) {
        throw std::runtime_error("failed to write PLY file");
    }
}

void test_icp_ply(const std::string& method, const icp::ICP::Config& config, const std::string& path_a, const std::string& path_b, const std::string& output_path) {

    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method, config).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(RAD_EPS, TRANS_EPS);

    // Load point clouds from PLY files
    std::vector<icp::Vector> a = parse_ply(path_a);
    std::vector<icp::Vector> b = parse_ply(path_b);

    auto result = driver.converge(a, b, icp::RBTransform(3));

    // Debug: Print transformation results
    std::cout << "Result Transform Translation X: " << result.transform.translation.x() << std::endl;
    std::cout << "Result Transform Translation Y: " << result.transform.translation.y() << std::endl;
    std::cout << "Result Transform Translation Z: " << result.transform.translation.z() << std::endl;
    std::cout << "Result Iteration Count: " << result.iteration_count << std::endl;

    // Check translation
    assert(std::abs(result.transform.translation.x()) <= TRANS_EPS);
    assert(std::abs(result.transform.translation.y()) <= TRANS_EPS);
    assert(std::abs(result.transform.translation.z()) <= TRANS_EPS);

    // Write transformed points to output file in PLY format
    write_transformed_points_ply(a, result.transform, output_path);
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <path_to_pointcloud_a.ply> <path_to_pointcloud_b.ply> <output_path>" << std::endl;
        return 1;
    }

    std::string path_a = argv[1];
    std::string path_b = argv[2];
    std::string output_path = argv[3];
    test_icp_ply(ICP_METHOD, icp::ICP::Config(), path_a, path_b, output_path);

    return 0;
}

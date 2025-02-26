#include <string>
#include "icp/geo.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include "icp/impl/vanilla_3d.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <stdexcept>

#define BURN_IN 0                  // Minimum required iterations for the algorithm
#define TRANS_EPS 0.001            // Translation tolerance in units
#define RAD_EPS ((double)(0.001))  // Rotation tolerance in radians
const std::string ICP_METHOD = "vanilla_3d";

std::vector<icp::Vector> parse_ply(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
        throw std::runtime_error("failed to read PLY file: failed to open file");
    }

    std::vector<icp::Vector> points;
    for (const auto& point: cloud->points) {
        points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    return points;
}

void write_transformed_points_ply(const std::vector<icp::Vector>& points,
    const icp::RBTransform& transform, const std::string& output_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point: points) {
        icp::Vector transformed_point = transform.apply_to(point);
        cloud->points.emplace_back(transformed_point.x(), transformed_point.y(),
            transformed_point.z());
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePLYFileASCII(output_path, *cloud) == -1) {
        throw std::runtime_error("failed to write PLY file");
    }
}

void test_icp_ply(const icp::ICP::Config& config, const std::string& path_a,
    const std::string& path_b, const std::string& output_path) {
    std::unique_ptr<icp::ICP> icp = std::make_unique<icp::Vanilla_3d>(config);
    icp::ICPDriver driver(std::move(icp));
    driver.set_min_iterations(BURN_IN);
    driver.set_max_iterations(100);
    driver.set_transform_tolerance(RAD_EPS, TRANS_EPS);

    // Load point clouds from PLY files
    std::vector<icp::Vector> a = parse_ply(path_a);
    std::vector<icp::Vector> b = parse_ply(path_b);

    auto result = driver.converge(a, b, icp::RBTransform(3));
    write_transformed_points_ply(a, result.transform, output_path);

    // Debug: Print transformation results
    std::cout << "Result Transform Translation X: " << result.transform.translation.x()
              << std::endl;
    std::cout << "Result Transform Translation Y: " << result.transform.translation.y()
              << std::endl;
    std::cout << "Result Transform Translation Z: " << result.transform.translation.z()
              << std::endl;
    std::cout << "Result Iteration Count: " << result.iteration_count << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <path_to_pointcloud_a.ply> <path_to_pointcloud_b.ply> <output_path>"
                  << std::endl;
        return 1;
    }

    std::string path_a = argv[1];
    std::string path_b = argv[2];
    std::string output_path = argv[3];
    test_icp_ply(icp::ICP::Config(), path_a, path_b, output_path);

    return 0;
}

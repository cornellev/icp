/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include <cstddef>
#include <string>
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "icp/geo.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include "icp/impl/trimmed_3d.h"

// Algorithm parameters
constexpr double TRANS_EPS = 0.0001;  // Translation tolerance (units)
constexpr double RAD_EPS = 0.0001;    // Rotation tolerance (radians)

/**
 * @brief Load a point cloud from PLY file
 *
 * @param path Path to the PLY file
 * @return PointCloud containing the loaded points
 * @throws std::runtime_error if file loading fails
 */
icp::PointCloud3 loadPointCloud(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
        throw std::runtime_error("Failed to read PLY file: " + path);
    }

    icp::PointCloud3 points(3, cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); i++) {
        auto point = cloud->points[i];
        points.col(static_cast<Eigen::Index>(i)) = Eigen::Vector3d(point.x, point.y, point.z);
    }

    return points;
}

/**
 * @brief Save a transformed point cloud to PLY file
 *
 * @param points Original point cloud
 * @param transform Transformation to apply
 * @param output_path Output file path
 * @throws std::runtime_error if file saving fails
 */
void saveTransformedPointCloud(const icp::PointCloud3& points, const icp::RBTransform3& transform,
    const std::string& output_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());

    icp::PointCloud3 transformed_cloud = transform * points;

    for (const auto& point: transformed_cloud.colwise()) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(point.x());
        pcl_point.y = static_cast<float>(point.y());
        pcl_point.z = static_cast<float>(point.z());
        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePLYFileASCII(output_path, *cloud) == -1) {
        throw std::runtime_error("Failed to write PLY file: " + output_path);
    }
}

/**
 * @brief Main function
 *
 * Parses command line arguments and runs the appropriate ICP algorithm.
 *
 * Usage:
 *   program <source.ply> <target.ply> <output.ply> [options]
 *
 */
int main(int argc, char* argv[]) {
    try {
        if (argc != 4) {
            return 1;
        }

        std::string path_a = argv[1];
        std::string path_b = argv[2];
        std::string output_path = argv[3];

        // ICP configuration
        icp::Config config;
        std::unique_ptr<icp::ICP3> icp = std::make_unique<icp::Trimmed3d>(config);
        icp::ICPDriver driver(std::move(icp));

        driver.set_max_iterations(100);
        driver.set_transform_tolerance(RAD_EPS, TRANS_EPS);
        // Load point clouds
        icp::PointCloud3 source_points = loadPointCloud(path_a);
        icp::PointCloud3 target_points = loadPointCloud(path_b);

        std::cout << "Starting ICP..." << "\n";

        // Run ICP
        auto result = driver.converge(source_points, target_points, icp::RBTransform3::Identity());

        // Save results
        saveTransformedPointCloud(source_points, result.transform, output_path);

        // Print results
        std::cout << "\n===== ICP Results =====" << "\n";
        std::cout << "  Rotation: \n" << result.transform.rotation() << "\n";
        std::cout << "  Translation: " << result.transform.translation().transpose() << "\n";
        std::cout << "  Iterations: " << result.iteration_count << "\n";

        std::cout << "ICP completed successfully" << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
/**
 * @file point_cloud_registration.cpp
 * @brief Point cloud registration using Iterative Closest Point (ICP) algorithm.
 *
 * This program implements both single-stage and multi-stage ICP for 3D point cloud
 * registration. It supports PLY file format for input and output point clouds.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <cassert>
#include <stdexcept>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <vector>
#include <Eigen/src/Geometry/Transform.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "icp/geo.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include "icp/impl/vanilla_3d.h"

// Algorithm parameters
#define BURN_IN 0                   // Minimum required iterations
#define TRANS_EPS 0.0001            // Translation tolerance (units)
#define RAD_EPS ((double)(0.0001))  // Rotation tolerance (radians)

/**
 * @brief Load a point cloud from PLY file
 *
 * @param path Path to the PLY file
 * @return PointCloud containing the loaded points
 * @throws std::runtime_error if file loading fails
 */
// std::vector<icp::Vector> loadPointCloud(const std::string& path) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
//         throw std::runtime_error("Failed to read PLY file: " + path);
//     }

//     std::vector<icp::Vector> points;
//     points.reserve(cloud->points.size());

//     for (const auto& point: cloud->points) {
//         points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
//     }

//     return points;
// }

// /**
//  * @brief Save a transformed point cloud to PLY file
//  *
//  * @param points Original point cloud
//  * @param transform Transformation to apply
//  * @param output_path Output file path
//  * @throws std::runtime_error if file saving fails
//  */
// void saveTransformedPointCloud(const std::vector<icp::Vector>& points,
//     const icp::RBTransform& transform, const std::string& output_path) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     cloud->points.reserve(points.size());

//     for (const auto& point: points) {
//         icp::Vector transformed_point = transform.apply_to(point);
//         cloud->points.emplace_back(transformed_point.x(), transformed_point.y(),
//             transformed_point.z());
//     }

//     cloud->width = cloud->points.size();
//     cloud->height = 1;
//     cloud->is_dense = true;

//     if (pcl::io::savePLYFileASCII(output_path, *cloud) == -1) {
//         throw std::runtime_error("Failed to write PLY file: " + output_path);
//     }
// }

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
    // if (argc != 4) {
    //     return 1;
    // }

    // std::string path_a = argv[1];
    // std::string path_b = argv[2];
    // std::string output_path = argv[3];

    // ICP configuration

    // driver.set_min_iterations(BURN_IN);
    // driver.set_max_iterations(100);
    // driver.set_transform_tolerance(RAD_EPS, TRANS_EPS);

    // Load point clouds
    // auto source_points = loadPointCloud(path_a);
    // auto target_points = loadPointCloud(path_b);
    //
    std::cout << "Start main" << std::endl;

    std::vector<icp::Vector> source_points;
    std::vector<icp::Vector> target_points;

    std::cout << "Starting ICP..." << std::endl;

    // Run ICP
    icp::ICPDriver driver;
    driver.converge(source_points, target_points, icp::RBTransform(3));

    // Save results
    // saveTransformedPointCloud(source_points, result.transform, output_path);
    // saveTransformedPointCloud(source_points, icp::RBTransform(3), output_path);

    // Print results
    // std::cout << "\n===== ICP Results =====" << std::endl;
    // std::cout << "  Rotation: \n" << result.transform.rotation << std::endl;
    // std::cout << "  Translation: [" << result.transform.translation.x() << ", "
    //           << result.transform.translation.y() << ", " << result.transform.translation.z() <<
    //           "]"
    //           << std::endl;
    // std::cout << "  Iterations: " << result.iteration_count << std::endl;

    std::cout << "ICP completed successfully" << std::endl;

    return 0;
}
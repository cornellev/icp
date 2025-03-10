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
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "icp/geo.h"
#include "icp/icp.h"
#include "icp/driver.h"
#include "icp/impl/vanilla_3d.h"

// Algorithm parameters
#define BURN_IN 10                  // Minimum required iterations
#define TRANS_EPS 0.0001            // Translation tolerance (units)
#define RAD_EPS ((double)(0.0001))  // Rotation tolerance (radians)

// Type definition for aligned vector of points
typedef std::vector<icp::Vector, Eigen::aligned_allocator<icp::Vector>> PointCloud;

/**
 * @brief Load a point cloud from PLY file
 *
 * @param path Path to the PLY file
 * @return PointCloud containing the loaded points
 * @throws std::runtime_error if file loading fails
 */
PointCloud loadPointCloud(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
        throw std::runtime_error("Failed to read PLY file: " + path);
    }

    PointCloud points;
    points.reserve(cloud->points.size());

    for (const auto& point: cloud->points) {
        points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
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
void saveTransformedPointCloud(const PointCloud& points, const icp::RBTransform& transform,
    const std::string& output_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());

    for (const auto& point: points) {
        icp::Vector transformed_point = transform.apply_to(point);
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(transformed_point.x());
        pcl_point.y = static_cast<float>(transformed_point.y());
        pcl_point.z = static_cast<float>(transformed_point.z());
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
 * @brief Calculate alignment cost between two point clouds
 *
 * @param source_points Source point cloud
 * @param target_points Target point cloud
 * @return double Average distance between nearest point pairs
 */
double calculateAlignmentCost(const PointCloud& source_points, const PointCloud& target_points) {
    if (source_points.empty() || target_points.empty()) {
        return std::numeric_limits<double>::max();
    }

    double total_cost = 0.0;
    int valid_points = 0;

    for (const auto& src_point: source_points) {
        double min_dist = std::numeric_limits<double>::max();

        // Find closest target point
        for (const auto& tgt_point: target_points) {
            double dist = (src_point - tgt_point).norm();
            min_dist = std::min(min_dist, dist);
        }

        if (min_dist < std::numeric_limits<double>::max()) {
            total_cost += min_dist;
            valid_points++;
        }
    }

    return (valid_points > 0) ? (total_cost / valid_points) : std::numeric_limits<double>::max();
}

/**
 * @brief Run multi-stage ICP alignment
 *
 * Each stage uses the result of the previous stage as the starting point.
 * The process stops when the improvement falls below the threshold or
 * the maximum number of stages is reached.
 *
 * @param config ICP configuration
 * @param path_a Source point cloud path
 * @param path_b Target point cloud path
 * @param output_path Output file path
 * @param max_stages Maximum number of stages
 * @param min_cost_improvement Minimum cost improvement to continue
 */
void runMultiStageICP(const icp::ICP::Config& config, const std::string& path_a,
    const std::string& path_b, const std::string& output_path, int max_stages = 5,
    double min_cost_improvement = 0.05) {
    // Load point clouds
    PointCloud current_points = loadPointCloud(path_a);
    PointCloud target_points = loadPointCloud(path_b);
    std::vector<icp::Vector> target_std(target_points.begin(), target_points.end());

    // Prepare transformation
    icp::RBTransform cumulative_transform(3);

    // Calculate initial cost
    double previous_cost = calculateAlignmentCost(current_points, target_points);
    std::cout << "Initial alignment cost: " << previous_cost << std::endl;

    // Multi-stage ICP loop
    int stage = 1;
    bool continue_processing = true;

    while (continue_processing && stage <= max_stages) {
        std::cout << "\n===== Stage " << stage << " =====" << std::endl;

        // Convert to standard vector for ICP processing
        std::vector<icp::Vector> current_std(current_points.begin(), current_points.end());

        // Setup ICP instance for this stage
        std::unique_ptr<icp::ICP> stage_icp = std::make_unique<icp::Vanilla_3d>(config);
        icp::ICPDriver stage_driver(std::move(stage_icp));
        stage_driver.set_min_iterations(BURN_IN);
        stage_driver.set_max_iterations(100);
        stage_driver.set_transform_tolerance(RAD_EPS, TRANS_EPS);

        // Run ICP for this stage
        auto result = stage_driver.converge(current_std, target_std, icp::RBTransform(3));

        // Update cumulative transform
        cumulative_transform = cumulative_transform.and_then(result.transform);

        // Generate stage output filename
        std::string stage_output = output_path;
        if (max_stages > 1) {
            size_t dot_pos = output_path.find_last_of('.');
            if (dot_pos != std::string::npos) {
                stage_output = output_path.substr(0, dot_pos) + "_stage" + std::to_string(stage)
                               + output_path.substr(dot_pos);
            } else {
                stage_output = output_path + "_stage" + std::to_string(stage);
            }
        }

        // Save stage result
        saveTransformedPointCloud(current_points, result.transform, stage_output);

        // Apply transform to get points for next stage
        PointCloud next_points;
        next_points.reserve(current_points.size());
        for (const auto& point: current_points) {
            next_points.push_back(result.transform.apply_to(point));
        }
        current_points = std::move(next_points);

        // Evaluate improvement
        double current_cost = calculateAlignmentCost(current_points, target_points);
        double relative_improvement = (previous_cost - current_cost) / previous_cost;

        // Print stage results
        std::cout << "  Translation: [" << result.transform.translation.x() << ", "
                  << result.transform.translation.y() << ", " << result.transform.translation.z()
                  << "]" << std::endl;
        std::cout << "  Iterations: " << result.iteration_count << std::endl;
        std::cout << "  Cost: " << current_cost << std::endl;
        std::cout << "  Improvement: " << (relative_improvement * 100) << "%" << std::endl;

        // Check if we should continue
        if (relative_improvement < min_cost_improvement) {
            std::cout << "Stopping: Improvement below threshold (" << (min_cost_improvement * 100)
                      << "%)" << std::endl;
            continue_processing = false;
        } else {
            previous_cost = current_cost;
            stage++;
        }
    }

    // Print final transform
    std::cout << "\n===== Final Results =====" << std::endl;
    std::cout << "  Cumulative translation: [" << cumulative_transform.translation.x() << ", "
              << cumulative_transform.translation.y() << ", "
              << cumulative_transform.translation.z() << "]" << std::endl;
    std::cout << "  Stages completed: " << (stage - 1) << std::endl;

    // Save final result with cumulative transform
    saveTransformedPointCloud(loadPointCloud(path_a), cumulative_transform, output_path);
}

/**
 * @brief Run enhanced single-stage ICP with stricter convergence criteria
 *
 * This version uses stricter convergence criteria to try to achieve
 * similar results to multi-stage ICP in a single pass.
 *
 * @param config ICP configuration
 * @param path_a Source point cloud path
 * @param path_b Target point cloud path
 * @param output_path Output file path
 */
void runEnhancedSingleStageICP(const icp::ICP::Config& config, const std::string& path_a,
    const std::string& path_b, const std::string& output_path) {
    // Create ICP instance with stricter termination conditions
    std::unique_ptr<icp::ICP> icp = std::make_unique<icp::Vanilla_3d>(config);
    icp::ICPDriver driver(std::move(icp));

    // Set stricter convergence criteria
    driver.set_min_iterations(10);
    driver.set_max_iterations(500);
    driver.set_transform_tolerance(RAD_EPS * 0.1, TRANS_EPS * 0.1);  // Reduced tolerance
    driver.set_relative_cost_tolerance(0.0001);                      // Strict cost tolerance

    // Load point clouds
    PointCloud source_points = loadPointCloud(path_a);
    PointCloud target_points = loadPointCloud(path_b);

    std::vector<icp::Vector> source_std(source_points.begin(), source_points.end());
    std::vector<icp::Vector> target_std(target_points.begin(), target_points.end());

    std::cout << "Starting enhanced single-stage ICP with stricter convergence criteria..."
              << std::endl;

    // Run ICP
    auto result = driver.converge(source_std, target_std, icp::RBTransform(3));

    // Save results
    saveTransformedPointCloud(source_points, result.transform, output_path);

    // Print results
    std::cout << "\n===== ICP Results =====" << std::endl;
    std::cout << "  Translation: [" << result.transform.translation.x() << ", "
              << result.transform.translation.y() << ", " << result.transform.translation.z() << "]"
              << std::endl;
    std::cout << "  Iterations: " << result.iteration_count << std::endl;
    std::cout << "  Final cost: " << result.cost << std::endl;
}

/**
 * @brief Main function
 *
 * Parses command line arguments and runs the appropriate ICP algorithm.
 *
 * Usage:
 *   program <source.ply> <target.ply> <output.ply> [options]
 *
 * Options:
 *   --single-stage        Use enhanced single-stage ICP
 *   --multi-stage N       Use multi-stage ICP with N stages maximum
 *   --threshold T         Set minimum cost improvement threshold to T
 */
int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <source.ply> <target.ply> <output.ply> [options]"
                  << std::endl;
        std::cerr << "Options:" << std::endl;
        std::cerr << "  --single-stage        Use enhanced single-stage ICP" << std::endl;
        std::cerr << "  --multi-stage N       Use multi-stage ICP with N stages (default: 5)"
                  << std::endl;
        std::cerr << "  --threshold T         Set improvement threshold to T (default: 0.05)"
                  << std::endl;
        return 1;
    }

    try {
        std::string path_a = argv[1];
        std::string path_b = argv[2];
        std::string output_path = argv[3];

        bool use_single_stage = false;
        int max_stages = 5;
        double min_cost_improvement = 0.05;

        // Parse options
        for (int i = 4; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--single-stage") {
                use_single_stage = true;
            } else if (arg == "--multi-stage" && i + 1 < argc) {
                max_stages = std::stoi(argv[++i]);
            } else if (arg == "--threshold" && i + 1 < argc) {
                min_cost_improvement = std::stod(argv[++i]);
            }
        }

        // ICP configuration
        icp::ICP::Config config;

        // Run appropriate ICP method
        if (use_single_stage) {
            std::cout << "Using enhanced single-stage ICP" << std::endl;
            runEnhancedSingleStageICP(config, path_a, path_b, output_path);
        } else {
            std::cout << "Using multi-stage ICP (max " << max_stages << " stages, threshold "
                      << (min_cost_improvement * 100) << "%)" << std::endl;
            runMultiStageICP(config, path_a, path_b, output_path, max_stages, min_cost_improvement);
        }

        std::cout << "ICP completed successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
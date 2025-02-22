#include <chrono>
#include <stdio.h>
#include <numeric>
#include <iostream>
#include <vector>

#include "icp/icp.h"
#include "icp/driver.h"
#include "icp/parse_scan.h"

struct BenchmarkResult {
    double min_cost;
    double max_cost;
    double median_cost;
    double mean_cost;
    size_t min_iterations;
    size_t max_iterations;
    size_t median_iterations;
    double mean_iterations;
    double average_time_per_invocation;
    double average_time_per_iteration;
};

struct BenchmarkParams {
    std::string method;
    size_t num_iter;
    size_t burn_in;
    double angle_tol;
    double trans_tol;
    uint32_t scan_id;
};

BenchmarkResult run_benchmark(BenchmarkParams params) {
    auto icp = icp::ICP::from_method(params.method).value();
    icp::ICPDriver driver(std::move(icp));
    driver.set_transform_tolerance(params.angle_tol, params.trans_tol);

    auto source = parse_lidar_scan("ex_data/scan" + std::to_string(params.scan_id) + "/first.csv");
    auto dest = parse_lidar_scan("ex_data/scan" + std::to_string(params.scan_id) + "/second.csv");

    std::vector<double> final_costs;
    std::vector<size_t> iteration_counts;

    const auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < params.num_iter; i++) {
        auto result = driver.converge(source, dest, icp::RBTransform());
        final_costs.push_back(result.cost);
        iteration_counts.push_back(result.iteration_count);
    }
    const auto end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<double> diff = end - start;

    std::sort(final_costs.begin(), final_costs.end());
    std::sort(iteration_counts.begin(), iteration_counts.end());

    BenchmarkResult result;
    result.min_cost = final_costs.front();
    result.max_cost = final_costs.back();
    result.median_cost = final_costs[final_costs.size() / 2];
    result.mean_cost = std::accumulate(final_costs.begin(), final_costs.end(), 0.0)
                       / final_costs.size();

    result.min_iterations = iteration_counts.front();
    result.max_iterations = iteration_counts.back();
    result.median_iterations = iteration_counts[iteration_counts.size() / 2];
    result.mean_iterations = std::accumulate(iteration_counts.begin(), iteration_counts.end(), 0.0)
                             / iteration_counts.size();

    result.average_time_per_invocation = diff.count() / params.num_iter;
    result.average_time_per_iteration =
        diff.count() / (std::accumulate(iteration_counts.begin(), iteration_counts.end(), 0.0));

    return result;
}

void print_benchmark_params(BenchmarkParams params) {
    std::cout << "* Method: " << params.method << "\n"
              << "* Number of iterations: " << params.num_iter << "\n"
              << "* Burn-in iterations: " << params.burn_in << "\n"
              << "* Angle tolerance: " << params.angle_tol << " rad\n"
              << "* Translation tolerance: " << params.trans_tol << " scan units\n";
}

void print_benchmark_result(BenchmarkResult result, BenchmarkParams params) {
    std::cout << "* Min cost: " << result.min_cost << "\n"
              << "* Max cost: " << result.max_cost << "\n"
              << "* Median cost: " << result.median_cost << "\n"
              << "* Mean cost: " << result.mean_cost << "\n"
              << "* Min iterations: " << result.min_iterations
              << " (real: " << (result.min_iterations - params.burn_in) << ")\n"
              << "* Max iterations: " << result.max_iterations
              << " (real: " << (result.max_iterations - params.burn_in) << ")\n"
              << "* Median iterations: " << result.median_iterations
              << " (real: " << (result.median_iterations - params.burn_in) << ")\n"
              << "* Mean iterations: " << result.mean_iterations
              << " (real: " << (result.mean_iterations - params.burn_in) << ")\n"
              << "* Average time per invocation: " << result.average_time_per_invocation << "s\n"
              << "* Average time per iteration: " << result.average_time_per_iteration << "s\n";
}

int main() {
    constexpr uint32_t scans = 3;
    constexpr size_t num_iter = 10;
    constexpr size_t burn_in = 0;
    constexpr double angle_tol = 0.1 * M_PI / 180;
    constexpr double trans_tol = 0.1;

    BenchmarkParams params;
    params.num_iter = num_iter;
    params.burn_in = burn_in;
    params.angle_tol = angle_tol;
    params.trans_tol = trans_tol;

    std::cout << "ICP ALGORITHM BENCHMARKING\n";

    for (const std::string& method: icp::ICP::registered_methods()) {
        std::cout << "=======================================\n";

        for (uint32_t scan_id = 1; scan_id <= scans; scan_id++) {
            if (scan_id != 1) std::cout << "---------------------------------------\n";

            params.method = method;
            params.scan_id = scan_id;

            print_benchmark_params(params);

            BenchmarkResult result = run_benchmark(params);

            print_benchmark_result(result, params);
        }
    }
    std::cout << "=======================================\n";
}

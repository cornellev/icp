#include <chrono>
#include <stdio.h>
#include <numeric>

#include "icp/icp.h"
#include "icp/driver.h"

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

BenchmarkResult run_benchmark(std::unique_ptr<icp::ICP> icp, const std::vector<icp::Vector>& source,
    const std::vector<icp::Vector>& destination) {
    std::cout << "ICP ALGORITHM BENCHMARKING\n";
    std::cout << "=======================================\n";

    constexpr size_t N = 50;
    constexpr size_t burn_in = 0;
    constexpr double angle_tol = 0.1 * M_PI / 180;
    constexpr double trans_tol = 0.1;

    std::cout << "* Method name: " << method << '\n';
    std::cout << "* Number of trials: " << N << '\n';
    std::cout << "* Burn-in period: " << burn_in << '\n';
    std::cout << "* Angle tolerance: " << angle_tol << " rad\n";
    std::cout << "* Translation tolerance: " << trans_tol << '\n';

    icp::ICPDriver driver(std::move(icp));
    driver.set_transform_tolerance(0.1 * M_PI / 180, 0.1);

    std::vector<double> final_costs;
    std::vector<size_t> iteration_counts;

    const auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < N; i++) {
        auto result = driver.converge(source, destination, icp::RBTransform());
        final_costs.push_back(result.cost);
        iteration_counts.push_back(result.iteration_count);
    }
    const auto end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<double> diff = end - start;

    double min_cost = final_costs.front();
    double max_cost = final_costs.back();
    double median_cost = final_costs[final_costs.size() / 2];
    double mean_cost = std::accumulate(final_costs.begin(), final_costs.end(), 0.0)
                       / final_costs.size();

    std::sort(iteration_counts.begin(), iteration_counts.end());

    size_t min_iterations = iteration_counts.front();
    size_t max_iterations = iteration_counts.back();
    size_t median_iterations = iteration_counts[iteration_counts.size() / 2];
    double mean_iterations = std::accumulate(iteration_counts.begin(), iteration_counts.end(), 0.0)
                             / iteration_counts.size();

    std::cout << "* Min cost: " << min_cost << "\n"
              << "* Max cost: " << max_cost << "\n"
              << "* Median cost: " << median_cost << "\n"
              << "* Mean cost: " << mean_cost << "\n";
    std::cout << "* Min iterations: " << min_iterations << " (real: " << (min_iterations - burn_in)
              << ")\n"
              << "* Max iterations: " << max_iterations << " (real: " << (max_iterations - burn_in)
              << ")\n"
              << "* Median iterations: " << median_iterations
              << " (real: " << (median_iterations - burn_in) << ")\n"
              << "* Mean iterations: " << mean_iterations
              << " (real: " << (mean_iterations - burn_in) << ")\n";
    std::cout << "* Average time per invocation: " << (diff.count() / N) << "s\n";
    std::cout << "* Average time per iteration: " << ((diff.count() / N) / mean_iterations)
              << "s\n";
}

int main() {}

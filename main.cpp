// Copyright (C) 2024 Ethan Uppal. All rights reserved.

#include <cassert>
#include <iostream>
extern "C" {
#include <cmdapp/cmdapp.h>
#include <config/config.h>
}
#include <chrono>
#include <numeric>
#include <algorithm>
#include "gui/window.h"
#include "sim/sim_config.h"
#include "sim/lidar_view.h"

struct LidarScan {
    double range_max;
    double range_min;
    double angle_min;
    double angle_max;
    double angle_increment;

    /** Units: centimeters */
    std::vector<icp::Vector> points;
};

void set_config_param(const char* var, const char* data, void* user_data) {
    if (strcmp(var, "n") == 0) {
        sim_config::n = std::stoi(data);
    } else if (strcmp(var, "scale") == 0) {
        sim_config::scale = std::stod(data);
    } else if (strcmp(var, "slope") == 0) {
        sim_config::slope = std::stod(data);
    } else if (strcmp(var, "intercept") == 0) {
        sim_config::intercept = std::stod(data);
    } else if (strcmp(var, "perturbation_range") == 0) {
        sim_config::perturbation_range = std::stod(data);
    } else if (strcmp(var, "x_displace") == 0) {
        sim_config::x_displace = std::stod(data);
    } else if (strcmp(var, "y_displace") == 0) {
        sim_config::y_displace = std::stod(data);
    } else if (strcmp(var, "angle_displace") == 0) {
        sim_config::angle_displace = std::stod(data);
    } else if (strcmp(var, "window_width") == 0) {
        sim_config::window_width = std::stoi(data);
    } else if (strcmp(var, "window_height") == 0) {
        sim_config::window_height = std::stoi(data);
    } else if (strcmp(var, "x_delta") == 0) {
        sim_config::x_delta = std::stoi(data);
    }
}

void parse_lidar_scan(const char* var, const char* data, void* user_data) {
    LidarScan* scan = static_cast<LidarScan*>(user_data);
    if (strcmp(var, "range_min") == 0) {
        scan->range_min = strtod(data, NULL);
    } else if (strcmp(var, "range_max") == 0) {
        scan->range_max = strtod(data, NULL);
    } else if (strcmp(var, "angle_max") == 0) {
        scan->angle_max = strtod(data, NULL);
        Log << "scan->angle_max = " << scan->angle_max << '\n';
    } else if (strcmp(var, "angle_min") == 0) {
        scan->angle_min = strtod(data, NULL);
        Log << "scan->angle_min = " << scan->angle_min << '\n';
    } else if (strcmp(var, "angle_increment") == 0) {
        scan->angle_increment = strtod(data, NULL);
    } else if (isnumber(var[0])) {
        long index = strtol(var, NULL, 10);
        double angle = scan->angle_min + index * scan->angle_increment;
        double range = strtod(data, NULL);
        if (range >= scan->range_min && range <= scan->range_max) {
            scan->points.push_back(icp::Vector(
                100 * range * std::cos(angle) + sim_config::window_width / 2,
                100 * range * std::sin(angle) + sim_config::window_height / 2));
        }
    }
}

void parse_config(const char* path, conf_parse_handler_t handler,
    void* user_data) {
    FILE* file = fopen(path, "r");
    if (!file) {
        perror("parse_config: fopen");
        std::exit(1);
    }

    if (conf_parse_file(file, handler, user_data) != 0) {
        perror("parse_config: conf_parse_file");
        std::exit(1);
    }

    fclose(file);
}

void launch_gui(LidarView* view, std::string visualized = "LiDAR scans") {
    Window window("Scan Matching", sim_config::window_width,
        sim_config::window_height);

    window.attach_view(view);

    std::cout << "SCAN MATCHING : ITERATIVE CLOSEST POINT\n";
    std::cout << "=======================================\n";
    std::cout << "* Visualizing: " << visualized << '\n';
    std::cout << "* Press the red <X> on the window to exit\n";
    std::cout << "* Press SPACE to toggle the simulation\n";
    std::cout << "* Press D to display the current transform\n";

    window.present();
}

void run_benchmark(const char* method, const LidarScan& source,
    const LidarScan& destination) {
    std::cout << "ICP ALGORITHM BENCHMARKING\n";
    std::cout << "=======================================\n";
    std::unique_ptr<icp::ICP> icp = icp::ICP::from_method(method);

    constexpr size_t N = 50;
    constexpr size_t burn_in = 20;
    constexpr double convergence_threshold = 20.0;

    std::cout << "* Number of trials: " << N << '\n';
    std::cout << "* Burn-in period: " << burn_in << '\n';
    std::cout << "* Ideal convergence threshold: " << convergence_threshold
              << '\n';

    std::vector<double> final_costs;
    std::vector<size_t> iteration_counts;

    const auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < N; i++) {
        icp->begin(source.points, destination.points, icp::RBTransform());
        icp::ICP::ConvergenceReport result = icp->converge(burn_in,
            convergence_threshold);
        final_costs.push_back(result.final_cost);
        iteration_counts.push_back(result.iteration_count);
    }
    const auto end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<double> diff = end - start;

    double min_cost = final_costs.front();
    double max_cost = final_costs.back();
    double median_cost = final_costs[final_costs.size() / 2];
    double mean_cost = std::accumulate(final_costs.begin(), final_costs.end(),
                           0.0)
                       / final_costs.size();

    std::sort(iteration_counts.begin(), iteration_counts.end());

    size_t min_iterations = iteration_counts.front();
    size_t max_iterations = iteration_counts.back();
    size_t median_iterations = iteration_counts[iteration_counts.size() / 2];
    double mean_iterations = std::accumulate(iteration_counts.begin(),
                                 iteration_counts.end(), 0.0)
                             / iteration_counts.size();

    std::cout << "* Min cost: " << min_cost << "\n"
              << "* Max cost: " << max_cost << "\n"
              << "* Median cost: " << median_cost << "\n"
              << "* Mean cost: " << mean_cost << "\n";
    std::cout << "* Min iterations: " << min_iterations
              << " (real: " << (min_iterations - burn_in) << ")\n"
              << "* Max iterations: " << max_iterations
              << " (real: " << (max_iterations - burn_in) << ")\n"
              << "* Median iterations: " << median_iterations
              << " (real: " << (median_iterations - burn_in) << ")\n"
              << "* Mean iterations: " << mean_iterations
              << " (real: " << (mean_iterations - burn_in) << ")\n";
    std::cout << "* Average time per invocation: " << (diff.count() / N)
              << "s\n";
}

int main(int argc, const char** argv) {
    if (ca_init(argc, argv) != 0) {
        perror("ca_init");
        return 1;
    }

    ca_description("Driver program for Ethan's scan-matching implementation.");
    ca_author("Ethan Uppal");
    ca_year(2024);
    ca_version(0, 0, 0);
    ca_versioning_info("All rights reserved.");

    ca_synopsis("[-h|-v]");
    ca_synopsis("-S FILE -D FILE [-l]");
    ca_synopsis("-b METHOD [-l]");

    bool* use_gui;
    bool* do_bench;
    bool* enable_log;
    bool* read_scan_files;
    bool* basic_mode;  // for gbody people
    const char* f_src;
    const char* f_dst;
    const char* config_file = "sim.conf";
    const char* method = "vanilla";

    assert(do_bench = ca_opt('b', "bench", "&SD", NULL,
               "benchmarks an ICP method (see -m). must pass -S/-D"));
    assert(read_scan_files = ca_opt('S', "src", ".FILE&D", &f_src,
               "source scan (pass with -D)"));
    assert(use_gui = ca_opt('g', "gui", "!@b", NULL, "visualizes ICP"));
    assert(ca_opt('D', "dst", ".FILE&S", &f_dst,
        "destination scan (pass with -S)"));
    assert(ca_opt('c', "config", ".FILE", &config_file,
        "selects a configuration file"));
    assert(ca_opt('m', "method", ".METHOD", &method, "selects an ICP method"));
    assert(basic_mode = ca_long_opt("basic-mode", "", NULL,
               "uses a ligher gui background"));
    assert(enable_log = ca_opt('l', "log", "", NULL, "enables debug logging"));
    assert(ca_opt('h', "help", "<h", NULL, "prints this info"));
    assert(ca_opt('v', "version", "<v", NULL, "prints version info"));

    if (argc == 1) {
        ca_print_help();
        return 1;
    } else if (ca_parse(NULL) != 0) {
        return 1;
    }

    Log.is_enabled = *enable_log;
    parse_config(config_file, set_config_param, NULL);
    if (*basic_mode) {
        sim_config::use_light_background = true;
    }

    if (!icp::ICP::is_registered_method(method)) {
        std::cerr << "error: unknown ICP method '" << method
                  << "'. expected one of:\n";
        for (const std::string& registered_method:
            icp::ICP::registered_methods()) {
            std::cerr << "* " << registered_method << '\n';
        }
        std::exit(1);
    }

    if (*read_scan_files) {
        LidarScan source, destination;
        parse_config(f_src, parse_lidar_scan, &source);
        parse_config(f_dst, parse_lidar_scan, &destination);
        if (*use_gui) {
            LidarView* view = new LidarView(source.points, destination.points,
                method);
            launch_gui(view,
                std::string(f_src) + std::string(" and ") + std::string(f_dst));
        } else if (*do_bench) {
            run_benchmark(method, source, destination);
        }
    }
}

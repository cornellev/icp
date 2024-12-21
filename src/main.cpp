// Copyright (C) 2024 Ethan Uppal. All rights reserved.

#include <iostream>
extern "C" {
#include <cmdapp/cmdapp.h>
#include <config/config.h>
}
#include <chrono>
#include <numeric>
#include <algorithm>
#include <sdlwrapper/gui/window.h>
#include <optional>
#include "sim/view_config.h"
#include "sim/lidar_view.h"
#include "icp/impl/vanilla.h"
#include "icp/impl/trimmed.h"
#include "icp/driver.h"

struct LidarScan {
    double range_max;
    double range_min;
    double angle_min;
    double angle_max;
    double angle_increment;

    /** Units: centimeters */
    std::vector<icp::Vector> points;
};

void set_config_param(const char* var, const char* data, [[maybe_unused]] void* user_data) {
    if (strcmp(var, "x_displace") == 0) {
        view_config::x_displace = std::stod(data);
    } else if (strcmp(var, "y_displace") == 0) {
        view_config::y_displace = std::stod(data);
    } else if (strcmp(var, "window_width") == 0) {
        view_config::window_width = std::stoi(data);
    } else if (strcmp(var, "window_height") == 0) {
        view_config::window_height = std::stoi(data);
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
    } else if (isdigit(var[0])) {
        long index = strtol(var, NULL, 10);
        double angle = scan->angle_min + index * scan->angle_increment;
        double range = strtod(data, NULL);
        if (range >= scan->range_min && range <= scan->range_max) {
            scan->points.push_back(icp::Vector(100 * range * std::cos(angle),
                100 * range * std::sin(angle)));
            // auto last = scan->points.back();
            // std::cerr << last.x() << ',' << last.y() << '\n';
        }
    }
}

void parse_config(const char* path, conf_parse_handler_t handler, void* user_data) {
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

void assert_opt(bool* opt_result) {
    if (!opt_result) {
        std::exit(1);
    }
}

void launch_gui(LidarView* view, std::string visualized = "LiDAR scans") {
    Window window("Scan Matching", view_config::window_width, view_config::window_height);

    window.attach_view(view);

    std::cout << "SCAN MATCHING : ITERATIVE CLOSEST POINT\n";
    std::cout << "=======================================\n";
    std::cout << "* Visualizing: " << visualized << '\n';
    std::cout << "* Press the red <X> on the window to exit\n";
    std::cout << "* Press SPACE to toggle the simulation\n";
    std::cout << "* Press D to display the current transform\n";
    std::cout << "* Press I to step forward a single iteration\n";

    window.present();
}

void run_benchmark(const char* method, std::unique_ptr<icp::ICP> icp, const LidarScan& source,
    const LidarScan& destination) {
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
        auto result = driver.converge(source.points, destination.points, icp::RBTransform());
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

int main(int argc, const char** argv) {
    if (ca_init(argc, argv) != 0) {
        perror("ca_init");
        return 1;
    }

    ca_description("Driver program for Ethan's ICP implementation.");
    ca_author("Ethan Uppal");
    ca_year(2024);
    ca_version(0, 0, 0);
    ca_versioning_info("All rights reserved.");

    ca_synopsis("[-h|-v]");
    ca_synopsis("-S FILE -D FILE [-l]");
    ca_synopsis("-b METHOD [-l]");

    bool* use_gui = NULL;
    bool* do_bench = NULL;
    bool* enable_log = NULL;
    bool* read_scan_files = NULL;
    bool* basic_mode = NULL;  // for gbody people
    const char* f_src = NULL;
    const char* f_dst = NULL;
    const char* config_file = "view.conf";
    const char* method = "vanilla";

    assert_opt(do_bench = ca_opt('b', "bench", "&SD", NULL,
                   "benchmarks an ICP method (see -m). must pass -S/-D"));
    assert_opt(read_scan_files = ca_opt('S', "src", ".FILE&D", &f_src,
                   "source scan (pass with -D)"));
    assert_opt(use_gui = ca_opt('g', "gui", "!@b", NULL, "visualizes ICP"));
    assert_opt(ca_opt('D', "dst", ".FILE&S", &f_dst, "destination scan (pass with -S)"));
    assert_opt(ca_opt('c', "config", ".FILE", &config_file,
        "selects a configuration file (default: view.conf)"));
    assert_opt(ca_opt('m', "method", ".METHOD", &method, "selects an ICP method"));
    assert_opt(basic_mode = ca_long_opt("basic-mode", "", NULL, "uses a ligher gui background"));
    assert_opt(enable_log = ca_opt('l', "log", "", NULL, "enables debug logging"));
    assert_opt(ca_opt('h', "help", "<h", NULL, "prints this info"));
    assert_opt(ca_opt('v', "version", "<v", NULL, "prints version info"));

    if (argc == 1) {
        ca_print_help();
        return 1;
    } else if (ca_parse(NULL) != 0) {
        return 1;
    }

    Log.is_enabled = *enable_log;
    parse_config(config_file, set_config_param, NULL);
    if (*basic_mode) {
        view_config::use_light_mode = true;
    }

    icp::ICP::register_builtin_methods();
    std::optional<std::unique_ptr<icp::ICP>> icp_opt = icp::ICP::from_method(method);

    if (!icp_opt.has_value()) {
        std::cerr << "error: unknown ICP method '" << method << "'. expected one of:\n";
        for (const std::string& registered_method: icp::ICP::registered_methods()) {
            std::cerr << "* " << registered_method << '\n';
        }
        std::exit(1);
    }

    std::unique_ptr<icp::ICP> icp = std::move(icp_opt.value());

    // std::vector<icp::Vector> a = {icp::Vector(0, 0), icp::Vector(100, 100)};
    // std::vector<icp::Vector> b = {};
    // double angle = (double)8 * M_PI / 180.0;
    // icp::Vector center = icp::get_centroid(a);
    // icp::Matrix rotation_matrix{
    //     {std::cos(angle), -std::sin(angle)}, {std::sin(angle), std::cos(angle)}};
    // for (const auto& point: a) {
    //     b.push_back(rotation_matrix * (point - center) + center);
    // }
    // LidarView* view = new LidarView(a, b, std::move(icp));
    // launch_gui(view, "test");
    // return 0;

    if (*read_scan_files) {
        LidarScan source, destination;
        parse_config(f_src, parse_lidar_scan, &source);
        parse_config(f_dst, parse_lidar_scan, &destination);

        if (*use_gui) {
            icp::ICP::Config config;
            config.set("overlap_rate", 0.9);
            LidarView* view = new LidarView(source.points, destination.points, std::move(icp));

            launch_gui(view, std::string(f_src) + std::string(" and ") + std::string(f_dst));
        } else if (*do_bench) {
            run_benchmark(method, std::move(icp), source, destination);
        }
    }
}

// Copyright (C) 2024 Ethan Uppal. All rights reserved.

#include <iostream>
#include <memory>
#include <optional>
extern "C" {
#include <cmdapp/cmdapp.h>
#include <config/config.h>
}
#include <sdlwrapper/gui/window.h>
#include "view_config.h"
#include "lidar_view.h"
#include "parse_scan.h"
#include "icp/icp.h"

void set_config_param(const char* var, const char* data, [[maybe_unused]] void* user_data) {
    if (strcmp(var, "x_displace") == 0) {
        view_config::x_displace = std::stod(data);
    } else if (strcmp(var, "y_displace") == 0) {
        view_config::y_displace = std::stod(data);
    } else if (strcmp(var, "window_width") == 0) {
        view_config::window_width = std::stoi(data);
    } else if (strcmp(var, "window_height") == 0) {
        view_config::window_height = std::stoi(data);
    } else if (strcmp(var, "view_scale") == 0) {
        view_config::view_scale = std::stod(data);
    }
}

void parse_config(const char* path) {
    FILE* file = fopen(path, "r");
    if (!file) {
        perror("parse_config: fopen");
        std::exit(1);
    }

    if (conf_parse_file(file, set_config_param, nullptr) != 0) {
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

    bool* enable_log = NULL;
    bool* read_scan_files = NULL;
    bool* basic_mode = NULL;  // for gbody people
    const char* f_src = NULL;
    const char* f_dst = NULL;
    const char* config_file = "view.conf";
    const char* method = "vanilla";

    assert_opt(read_scan_files = ca_opt('S', "src", ".FILE&D", &f_src,
                   "source scan (pass with -D)"));
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
    parse_config(config_file);
    if (*basic_mode) {
        view_config::use_light_mode = true;
    }

    std::optional<std::unique_ptr<icp::ICP2>> icp_opt = icp::ICP2::from_method(method,
        icp::Config());

    // std::optional<std::unique_ptr<icp::ICP3>> icp_opt2 = icp::ICP3::from_method(method,
    //     icp::Config());

    if (!icp_opt.has_value()) {
        std::cerr << "error: unknown ICP method '" << method << "'. expected one of:\n";
        for (const std::string& registered_method: icp::ICP2::registered_methods()) {
            std::cerr << "* " << registered_method << '\n';
        }
        std::exit(1);
    }

    std::unique_ptr<icp::ICP2> icp = std::move(icp_opt.value());

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
        auto source = parse_lidar_scan(f_src);
        auto dest = parse_lidar_scan(f_dst);

        icp::Config config;
        config.set("overlap_rate", 0.9);
        LidarView* view = new LidarView(source, dest, std::move(icp));

        launch_gui(view, std::string(f_src) + std::string(" and ") + std::string(f_dst));
        delete view;
    }
}

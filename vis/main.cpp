/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * SPDX-License-Identifier: MIT
 */

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

#define ASSERT_OPT(result)                                                                         \
    do {                                                                                           \
        if ((result) == nullptr) {                                                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0);

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

bool parse_config(const char* path) {
    FILE* file = fopen(path, "r");
    if (file == nullptr) {
        perror("parse_config: fopen");
        fclose(file);
        return false;
    }

    if (conf_parse_file(file, set_config_param, nullptr) != 0) {
        perror("parse_config: conf_parse_file");
        fclose(file);
        return false;
    }

    fclose(file);
    return true;
}

void launch_gui(LidarView* view, const std::string& visualized = "LiDAR scans") {
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

    ca_description("Driver program for CEV's ICP implementation.");
    ca_author("Ethan Uppal");
    ca_author("Cornell Electric Vehicles");
    ca_year(2025);
    ca_version(0, 0, 0);
    ca_versioning_info("See LICENSE for details.");

    ca_synopsis("[-h|-v]");
    ca_synopsis("-S FILE -D FILE [-l]");
    ca_synopsis("-b METHOD [-l]");

    bool* enable_log = nullptr;
    bool* read_scan_files = nullptr;
    bool* basic_mode = nullptr;  // for gbody people
    const char* f_src = nullptr;
    const char* f_dst = nullptr;
    const char* config_file = "view.conf";
    const char* method = "vanilla";

    // NOLINTBEGIN(bugprone-assignment-in-if-condition)
    ASSERT_OPT(read_scan_files = ca_opt('S', "src", ".FILE&D", &f_src,
                   "source scan (pass with -D)"));
    ASSERT_OPT(ca_opt('D', "dst", ".FILE&S", &f_dst, "destination scan (pass with -S)"));
    ASSERT_OPT(ca_opt('c', "config", ".FILE", &config_file,
        "selects a configuration file (default: view.conf)"));
    ASSERT_OPT(ca_opt('m', "method", ".METHOD", &method, "selects an ICP method"));
    ASSERT_OPT(basic_mode = ca_long_opt("basic-mode", "", nullptr, "uses a ligher gui background"));
    ASSERT_OPT(enable_log = ca_opt('l', "log", "", nullptr, "enables debug logging"));
    ASSERT_OPT(ca_opt('h', "help", "<h", nullptr, "prints this info"));
    ASSERT_OPT(ca_opt('v', "version", "<v", nullptr, "prints version info"));
    // NOLINTEND(bugprone-assignment-in-if-condition)

    if (argc == 1) {
        ca_print_help();
        return 1;
    } else if (ca_parse(nullptr) != 0) {
        return 1;
    }

    Log.is_enabled = *enable_log;
    bool success = parse_config(config_file);
    if (!success) {
        return 1;
    }

    if (*basic_mode) {
        view_config::use_light_mode = true;
    }

    std::optional<std::unique_ptr<icp::ICP2>> icp_opt = icp::ICP2::from_method(method,
        icp::Config());

    if (!icp_opt.has_value()) {
        std::cerr << "error: unknown ICP method '" << method << "'. expected one of:\n";
        for (const std::string& registered_method: icp::ICP2::registered_methods()) {
            std::cerr << "* " << registered_method << '\n';
        }
        return 1;
    }

    std::unique_ptr<icp::ICP2> icp = std::move(icp_opt.value());

    if (*read_scan_files) {
        auto source = parse_lidar_scan(f_src);
        auto dest = parse_lidar_scan(f_dst);

        icp::Config config;
        config.set("overlap_rate", 0.9);
        auto* view = new LidarView(source, dest, std::move(icp));

        launch_gui(view, std::string(f_src) + std::string(" and ") + std::string(f_dst));
    }
}

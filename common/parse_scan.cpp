/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#include "parse_scan.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include <vector>
#include "icp/geo.h"

std::vector<icp::Vector> parse_lidar_scan(std::string path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("failed to read lidar scan: failed to open file");
    }

    std::vector<icp::Vector> result;

    std::string line;
    while (std::getline(in, line)) {
        size_t index = line.find(',');
        std::string_view view(line);

        std::string_view x_view = view.substr(0, index);
        std::string_view y_view = view.substr(index + 1);

        char* end = nullptr;
        double x = std::strtod(x_view.cbegin(), &end);
        double y = std::strtod(y_view.cbegin(), &end);
        if (std::isinf(x) || std::isinf(y)) {
            continue;
        }

        result.emplace_back(x, y);
    }

    return result;
}

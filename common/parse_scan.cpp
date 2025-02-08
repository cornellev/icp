#include "parse_scan.h"
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include <vector>
#include "icp/geo.h"

float parse_float(std::string_view view) {
    float f;
    auto [_, ec] = std::from_chars(view.data(), (view.data() + view.size()), f);
    if (ec != std::errc()) {
        throw std::runtime_error("failed to read lidar scan: invalid value");
    }
    return f;
}

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

        float x = parse_float(x_view);
        float y = parse_float(y_view);
        if (std::isinf(x) || std::isinf(y)) {
            continue;
        }

        result.emplace_back(x, y);
    }

    return result;
}

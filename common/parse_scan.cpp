#include "parse_scan.h"

void parse_scan_var(const char* var, const char* data, void* user_data) {
    LidarScan* scan = static_cast<LidarScan*>(user_data);
    if (strcmp(var, "range_min") == 0) {
        scan->range_min = strtod(data, NULL);
    } else if (strcmp(var, "range_max") == 0) {
        scan->range_max = strtod(data, NULL);
    } else if (strcmp(var, "angle_max") == 0) {
        scan->angle_max = strtod(data, NULL);
    } else if (strcmp(var, "angle_min") == 0) {
        scan->angle_min = strtod(data, NULL);
    } else if (strcmp(var, "angle_increment") == 0) {
        scan->angle_increment = strtod(data, NULL);
    } else if (isdigit(var[0])) {
        long index = strtol(var, NULL, 10);
        double angle = scan->angle_min + index * scan->angle_increment;
        double range = strtod(data, NULL);
        if (range >= scan->range_min && range <= scan->range_max) {
            scan->points.push_back(icp::Vector(100 * range * std::cos(angle),
                100 * range * std::sin(angle)));
        }
    }
}

LidarScan parse_lidar_scan(const char* path) {
    FILE* file = fopen(path, "r");
    if (!file) {
        perror("parse_config: fopen");
        std::exit(1);
    }

    LidarScan scan;
    if (conf_parse_file(file, parse_scan_var, &scan) != 0) {
        perror("parse_config: conf_parse_file");
        std::exit(1);
    }

    fclose(file);

    return scan;
}

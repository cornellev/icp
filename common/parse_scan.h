#include <vector>
#include "icp/geo.h"
extern "C" {
#include <config/config.h>
}

struct LidarScan {
    double range_max;
    double range_min;
    double angle_min;
    double angle_max;
    double angle_increment;

    std::vector<icp::Vector> points;
};

LidarScan parse_lidar_scan(const char* path);

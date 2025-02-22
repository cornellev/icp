#include "icp/parse_ply.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <stdexcept>

std::vector<icp::Vector> parse_ply(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
        throw std::runtime_error("failed to read PLY file: failed to open file");
    }

    std::vector<icp::Vector> points;
    for (const auto& point: cloud->points) {
        points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    return points;
}

/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "icp/geo.h"

icp::PointCloud2 parse_lidar_scan(const std::string& path);

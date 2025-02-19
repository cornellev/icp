#pragma once

#include <vector>
#include <string>
#include "icp/geo.h"

std::vector<icp::Vector> parse_ply(const std::string& path);

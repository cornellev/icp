// Copyright (C) 2024 Ethan Uppal. All rights reserved.

#include <cmath>
#include "icp.h"

namespace icp {
    ICP::ICP(const std::vector<icp::Point>& a, const std::vector<icp::Point>& b,
        double rate)
        : a(a), b(b), rate(rate), pair(a.size()), dist(a.size()) {}

    void ICP::iterate() {
        size_t n = a.size();
        size_t m = b.size();

        // point-to-point matching O(n^2)
        for (size_t i = 0; i < n; i++) {
            dist[i] = INFINITY;
            for (size_t j = 0; j < m; j++) {
                double alt = std::hypot(a[i].x - b[j].x, a[i].y - b[j].y);
                if (alt < dist[i]) {
                    pair[i] = j;
                    dist[i] = alt;
                }
            }
        }

        // gradient descent
        double dx_effect = 0;
        double dy_effect = 0;
        for (size_t i = 0; i < n; i++) {
            // replace with gradient of L
            dx_effect += (a[i].x + t.dx - b[pair[i]].x) * 2 / n;
            dy_effect += (a[i].y + t.dy - b[pair[i]].y) * 2 / n;
        }
        t.dx -= dx_effect * rate;
        t.dy -= dy_effect * rate;

        // compute new cost
        previous_cost = current_cost;
        current_cost = 0;
        for (size_t i = 0; i < n; i++) {
            // replace with your L
            double cx = a[i].x + t.dx - b[pair[i]].x;
            double cy = a[i].y + t.dy - b[pair[i]].y;
            current_cost += (cx * cx + cy * cy) / n;
        }
    }

    void ICP::converge(double convergence_threshold) {
        while (current_cost > convergence_threshold
               && (current_cost < previous_cost || current_cost == INFINITY)) {
            iterate();
        }
    }

    const std::vector<icp::Point>& ICP::source() const {
        return a;
    }
    const std::vector<icp::Point>& ICP::destination() const {
        return b;
    }

    double ICP::cost() const {
        return current_cost;
    }

    const Transform& ICP::transform() const {
        return t;
    }
}

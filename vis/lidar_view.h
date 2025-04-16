/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <SDL.h>
#include <vector>
#include <sdlwrapper/gui/view.h>
#include <sdlwrapper/util/keyboard.h>
#include <memory>
#include "icp/geo.h"
#include "icp/icp.h"

class LidarView final : public View {
    icp::PointCloud2 source;
    icp::PointCloud2 destination;
    std::unique_ptr<icp::ICP2> icp;
    Keyboard keyboard;
    bool is_iterating;
    size_t iterations;

    void step();

public:
    /** Constructs a new lidar view visualizing ICP (by method `method`) on
     * the given instance (`source` and `destination`). */
    LidarView(icp::PointCloud2 source, icp::PointCloud2 destination,
        std::unique_ptr<icp::ICP2> icp);

    void on_event(const SDL_Event& event) override;
    void draw(SDL_Renderer* renderer, const SDL_Rect* frame, double dtime) override;
};
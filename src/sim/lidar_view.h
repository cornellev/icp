/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <SDL.h>
#include <vector>
#include "gui/view.h"
#include "util/keyboard.h"
#include "icp/icp.h"

class LidarView final : public View {
    std::vector<icp::Vector> source;
    std::vector<icp::Vector> destination;
    std::unique_ptr<icp::ICP> icp;
    Keyboard keyboard;
    bool is_iterating;
    size_t iterations;

    void step();

public:
    /** Constructs a new lidar view visualizing ICP (by method `method`) on
     * the given instance (`source` and `destination`). */
    LidarView(std::vector<icp::Vector> source,
        std::vector<icp::Vector> destination, const std::string method,
        const icp::ICP::Config& config = icp::ICP::Config());

    ~LidarView() noexcept override;
    void on_event(const SDL_Event& event) override;
    void draw(SDL_Renderer* renderer, const SDL_Rect* frame,
        double dtime) override;
};

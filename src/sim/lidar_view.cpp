/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include <random>
#include <sdlwrapper/util/logger.h>
#include <sdlwrapper/util/keyboard.h>
#include <sdlwrapper/geo/midpoint.h>
#include "lidar_view.h"
#include "view_config.h"

#define CIRCLE_RADIUS 3

LidarView::LidarView(std::vector<icp::Vector> source, std::vector<icp::Vector> destination,
    std::unique_ptr<icp::ICP> icp)
    : source(source),
      destination(destination),
      icp(std::move(icp)),
      keyboard(false),
      is_iterating(false),
      iterations(0) {
    this->icp->begin(source, destination, icp::RBTransform());
}

LidarView::~LidarView() noexcept {
    icp.release();
}

void LidarView::step() {
    icp->iterate();
    iterations++;
}

void LidarView::on_event(const SDL_Event& event) {
    bool space_before = keyboard.query(SDLK_SPACE);
    bool d_before = keyboard.query(SDLK_d);
    bool i_before = keyboard.query(SDLK_i);
    keyboard.update(event);
    bool space_after = keyboard.query(SDLK_SPACE);
    bool d_after = keyboard.query(SDLK_d);
    bool i_after = keyboard.query(SDLK_i);

    if (!space_before && space_after) {
        is_iterating = !is_iterating;
    }
    if (!i_before && i_after) {
        step();
    }
    if (!d_before && d_after) {
        std::cerr << "DEBUG PRINT:\n";
        std::cerr << "icp->current_transform() = " << icp->current_transform().to_string() << '\n';
        std::cerr << "icp->calculate_cost() = " << icp->calculate_cost() << '\n';
        std::cerr << "iterations = " << iterations << '\n';
    }
}

void LidarView::draw(SDL_Renderer* renderer, [[maybe_unused]] const SDL_Rect* frame,
    [[maybe_unused]] double dtime) {
    if (view_config::use_light_background) {
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    } else {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    }
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
    for (const icp::Vector& point: destination) {
        SDL_DrawCircle(renderer, point[0] + view_config::x_displace,
            point[1] + view_config::y_displace, CIRCLE_RADIUS);
    }

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    for (const icp::Vector& point: source) {
        icp::Vector result = icp->current_transform().apply_to(point);
        SDL_DrawCircle(renderer, result[0] + view_config::x_displace,
            result[1] + view_config::y_displace, CIRCLE_RADIUS);
    }

    icp::Vector a_cm = icp->current_transform().apply_to(icp::get_centroid(source));
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 10);
    SDL_DrawCircle(renderer, a_cm.x() + view_config::x_displace, a_cm.y() + view_config::y_displace,
        20);

    icp::Vector b_cm = icp::get_centroid(destination);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 10);
    SDL_DrawCircle(renderer, b_cm.x() + view_config::x_displace, b_cm.y() + view_config::y_displace,
        20);

    if (is_iterating) {
        step();
    }
}

/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * SPDX-License-Identifier: MIT
 */

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <SDL_pixels.h>
#include <sdlwrapper/util/logger.h>
#include <sdlwrapper/util/keyboard.h>
#include <sdlwrapper/geo/midpoint.h>
#include <Eigen/Geometry>
#include "lidar_view.h"
#include "icp/geo.h"
#include "view_config.h"

constexpr int CIRCLE_RADIUS = 3;

LidarView::LidarView(const icp::PointCloud2& source, const icp::PointCloud2& destination,
    std::unique_ptr<icp::ICP2> icp)
    : source(source),
      destination(destination),
      icp(std::move(icp)),
      keyboard(false),
      is_iterating(false),
      iterations(0) {
    this->icp->begin(source, destination, icp::RBTransform2::Identity());
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
        std::cerr << "icp->current_transform().rotation() = "
                  << Eigen::Rotation2Dd(icp->current_transform().rotation()).angle() << '\n';
        std::cerr << "icp->current_transform().translation() = "
                  << icp->current_transform().translation().transpose() << '\n';
        std::cerr << "icp->calculate_cost() = " << icp->calculate_cost() << '\n';
        std::cerr << "iterations = " << iterations << '\n';
    }
}

// this is the correct logic if we want the green line to be the indicator of how good is our
// current transformation
void LidarView::draw_matches(SDL_Renderer* renderer) {
    const auto& matches = icp->get_matches();  // one iteration before we calculated the transform
                                               //
    double max_cost = std::max_element(matches.begin(), matches.end(),
        [](const auto& first, const auto& second) {
            return first.cost < second.cost;
        })->cost;

    // the size of available points in "a" changed
    for (Eigen::Index i = 0; i < source.cols(); i++) {
        const auto& source_point = source.col(matches[i].point);
        const auto& destination_point = destination.col(matches[i].pair);
        auto transformed_source = icp->current_transform() * source_point;

        SDL_SetRenderDrawColor(renderer, 0,
            255 - static_cast<int>(matches[i].cost / max_cost * 255), 0, SDL_ALPHA_OPAQUE);

        // current transform
        SDL_RenderDrawLine(renderer,
            static_cast<int>(view_config::view_scale * transformed_source[0]
                             + view_config::x_displace),
            static_cast<int>(view_config::view_scale * transformed_source[1]
                             + view_config::y_displace),
            static_cast<int>(view_config::view_scale * destination_point[0]
                             + view_config::x_displace),
            static_cast<int>(view_config::view_scale * destination_point[1]
                             + view_config::y_displace));
    }
}

void LidarView::draw(SDL_Renderer* renderer, [[maybe_unused]] const SDL_Rect* frame,
    [[maybe_unused]] double dtime) {
    if (view_config::use_light_mode) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    } else {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    }
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
    for (const icp::Vector2& point: destination.colwise()) {
        SDL_DrawCircle(renderer,
            static_cast<int>(view_config::view_scale * point.x() + view_config::x_displace),
            static_cast<int>(view_config::view_scale * point.y() + view_config::y_displace),
            CIRCLE_RADIUS);
    }

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    for (const icp::Vector2& point: source.colwise()) {
        icp::Vector2 result = icp->current_transform() * point;
        SDL_DrawCircle(renderer,
            static_cast<int>(view_config::view_scale * result[0] + view_config::x_displace),
            static_cast<int>(view_config::view_scale * result[1] + view_config::y_displace),
            CIRCLE_RADIUS);
    }

    // Draw a line connecting the transformed source point to the destination point (in green)
    draw_matches(renderer);

    icp::Vector2 a_cm = icp->current_transform() * icp::get_centroid(source);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_DrawCircle(renderer,
        static_cast<int>(view_config::view_scale * a_cm.x() + view_config::x_displace),
        static_cast<int>(view_config::view_scale * a_cm.y() + view_config::y_displace), 20);

    icp::Vector2 b_cm = icp::get_centroid(destination);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
    SDL_DrawCircle(renderer,
        static_cast<int>(view_config::view_scale * b_cm.x() + view_config::x_displace),
        static_cast<int>(view_config::view_scale * b_cm.y() + view_config::y_displace), 20);

    if (is_iterating) {
        step();
    }
}
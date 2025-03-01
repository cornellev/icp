/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
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

void LidarView::draw_matches(SDL_Renderer* renderer) {
    // if (!icp || source.empty() || destination.empty()) return;
    const auto& matches = icp->get_matches();  // one iteration before we calculated the transform
    // if (matches.size() != source.size()) return;

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
    // the size of available points in "a" changed
    for (size_t i = 0; i < source.size(); i++) {
        // if (matches[i].pair >= destination.size()) continue;
        const auto& source_point = source[matches[i].point];
        const auto& destination_point = destination[matches[i].pair];
        auto transformed_source = icp->current_transform().apply_to(source_point);

        // current transform
        SDL_RenderDrawLine(renderer,
            static_cast<int>(view_config::view_scale * transformed_source[0])
                + view_config::x_displace,
            static_cast<int>(view_config::view_scale * transformed_source[1])
                + view_config::y_displace,
            static_cast<int>(view_config::view_scale * destination_point[0])
                + view_config::x_displace,
            static_cast<int>(view_config::view_scale * destination_point[1])
                + view_config::y_displace);
    }
}

void LidarView::draw_next_matches(SDL_Renderer* renderer) {
    const auto& next_matches = icp->get_next_matches();

    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 128);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    for (size_t i = 0; i < source.size(); i++) {
        auto predicted_transform = icp->current_transform().and_then(icp->get_last_step());
        auto predicted_source = predicted_transform.apply_to(source[next_matches[i].point]);

        const auto& destination_point = destination[next_matches[i].pair];

        SDL_RenderDrawLine(renderer,
            static_cast<int>(view_config::view_scale * predicted_source[0])
                + view_config::x_displace,
            static_cast<int>(view_config::view_scale * predicted_source[1])
                + view_config::y_displace,
            static_cast<int>(view_config::view_scale * destination_point[0])
                + view_config::x_displace,
            static_cast<int>(view_config::view_scale * destination_point[1])
                + view_config::y_displace);
    }
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
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
    for (const icp::Vector& point: destination) {
        SDL_DrawCircle(renderer, view_config::view_scale * point[0] + view_config::x_displace,
            view_config::view_scale * point[1] + view_config::y_displace, CIRCLE_RADIUS);
    }

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    for (const icp::Vector& point: source) {
        icp::Vector result = icp->current_transform().apply_to(point);
        SDL_DrawCircle(renderer, view_config::view_scale * result[0] + view_config::x_displace,
            view_config::view_scale * result[1] + view_config::y_displace, CIRCLE_RADIUS);
    }

    // Draw a line connecting the transformed source point to the destination point (in green)
    draw_matches(renderer);

    if (view_config::show_prediction) {
        draw_next_matches(renderer);
    }

    icp::Vector a_cm = icp->current_transform().apply_to(icp::get_centroid(source));
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_DrawCircle(renderer, view_config::view_scale * a_cm.x() + view_config::x_displace,
        view_config::view_scale * a_cm.y() + view_config::y_displace, 20);

    icp::Vector b_cm = icp::get_centroid(destination);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, SDL_ALPHA_OPAQUE);
    SDL_DrawCircle(renderer, view_config::view_scale * b_cm.x() + view_config::x_displace,
        view_config::view_scale * b_cm.y() + view_config::y_displace, 20);

    if (is_iterating) {
        step();
    }
}
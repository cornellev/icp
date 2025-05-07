/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <optional>
#include <chrono>
#include <memory>
#include <utility>
#include "icp/geo.h"
#include "icp/icp.h"

namespace icp {

    /**
     * @brief Driver for running ICP to convergence.
     */
    template<const Dimension Dim>
    class ICPDriver {
    public:
        /** The result of running `ICPDriver::converge`. */
        struct ConvergenceState {
            /** The cost achieved. */
            double cost = 0;

            /** The number of iterations performed. */
            size_t iteration_count = 0;

            /** The transform. */
            RBTransform<Dim> transform = RBTransform<Dim>::Identity();
        };

        /**
         * @brief Constructs a new ICPDriver using the given ICP method.
         *
         * @param icp The ICP method to use.
         */
        ICPDriver(std::unique_ptr<ICP<Dim>> icp): icp_(std::move(icp)) {}

        /**
         * @brief Runs ICP to convergence based on the termination conditions set. If no conditions
         * are set, ICP will run indefinitely. This is rarely desirable.
         *
         * @param source The source point cloud.
         * @param target The target point cloud.
         * @param guess The initial guess for the transformation.
         * @return ConvergenceState
         */
        ConvergenceState converge(const PointCloud<Dim>& source, const PointCloud<Dim>& target,
            RBTransform<Dim> guess) {
            start_time_ = std::chrono::steady_clock::now();
            icp_->begin(source, target, guess);
            ConvergenceState state;

            state.iteration_count = 0;
            state.cost = icp_->calculate_cost();
            state.transform = icp_->current_transform();

            std::optional<ConvergenceState> last_state{};

            while (!should_terminate(state, last_state)) {
                last_state = state;
                icp_->iterate();
                state.iteration_count++;
                state.cost = icp_->calculate_cost();
                state.transform = icp_->current_transform();
            }

            return state;
        }

        /**
         * @brief Sets the minimum number of iterations to run. This number of iterations will
         * always be performed.
         *
         * @param min_iterations The minimum number of iterations to run.
         */
        void set_min_iterations(uint64_t min_iterations) {
            assert(!max_iterations_ || min_iterations <= max_iterations_.value());
            min_iterations_ = min_iterations;
        }

        /**
         * @brief Sets the maximum number of iterations to run.
         *
         * @param max_iterations The maximum number of iterations to run.
         */
        void set_max_iterations(uint64_t max_iterations) {
            assert(!min_iterations_ || max_iterations >= min_iterations_.value());
            max_iterations_ = max_iterations;
        }

        /**
         * @brief Sets the cost at which to stop ICP. `converge` will return when a cost below
         * `stop_cost` is achieved.
         *
         * @param stop_cost The cost at which to stop ICP.
         */
        void set_stop_cost(double stop_cost) {
            stop_cost_ = stop_cost;
        }

        // TODO: fix docs to use math once doxygen is fixed
        /**
         * @brief Sets the relative cost tolerance. `converge` will return when the cost
         * changes by less than this fraction of the current cost, i.e. when |`current_cost` -
         * `prev_cost`| / < `relative_cost_tolerance`.
         *
         * @param relative_cost_tolerance The relative cost tolerance.
         */
        void set_relative_cost_tolerance(double relative_cost_tolerance) {
            relative_cost_tolerance_ = relative_cost_tolerance;
        }

        /**
         * @brief Set the absolute cost tolerance. `converge` will return when the cost changes by
         * less than this amount, i.e. when |`current_cost` - `prev_cost`|  <
         * `absolute_cost_tolerance`.
         *
         * @param absolute_cost_tolerance The absolute cost tolerance.
         */
        void set_absolute_cost_tolerance(double absolute_cost_tolerance) {
            absolute_cost_tolerance_ = absolute_cost_tolerance;
        }

        /**
         * @brief Set the transform tolerance. `converge` will return when both the angle and
         * translation tolerances are met.
         *
         * @param angle_tolerance The angle tolerance in radians. The tolerance is met when the
         * angle of rotation between the current and previous transformation around the axis of
         * rotation is less than `angle_tolerance`.
         * @param translation_tolerance The translation tolerance in scan units. The tolerance is
         * met when the change in translation is less than `translation_tolerance`.
         */
        void set_transform_tolerance(double angle_tolerance, double translation_tolerance) {
            angle_tolerance_rad_ = angle_tolerance;
            translation_tolerance_ = translation_tolerance;
        }

        /**
         * @brief Set the time limit for `converge`. Note that `coverge` may take slightly longer
         * than this time (up to 1 iteration time) to return.
         *
         * @param time_limit The time limit.
         */
        void set_time_limit(std::chrono::duration<double> time_limit) {
            time_limit_ = time_limit;
        }

    private:
        bool should_terminate(ConvergenceState current_state,
            std::optional<ConvergenceState> last_state) {
            // absolute conditions based only on current state
            if (min_iterations_ && current_state.iteration_count < min_iterations_.value()) {
                return false;
            }

            if (max_iterations_ && current_state.iteration_count >= max_iterations_.value()) {
                return true;
            }

            if (stop_cost_ && current_state.cost < stop_cost_.value()) {
                return true;
            }

            if (time_limit_) {
                auto current_time = std::chrono::steady_clock::now();
                if (current_time - start_time_ > time_limit_.value()) {
                    return true;
                }
            }

            // end if we don't have a last state
            if (!last_state) {
                return false;
            }

            // relative conditions based on progress
            double delta_cost = current_state.cost - last_state.value().cost;
            if (absolute_cost_tolerance_
                && std::abs(delta_cost) < absolute_cost_tolerance_.value()) {
                return true;
            }

            double relative_cost_change = std::abs(delta_cost) / current_state.cost;
            if (relative_cost_tolerance_
                && relative_cost_change < relative_cost_tolerance_.value()) {
                return true;
            }

            // TODO: is this metric right
            if (angle_tolerance_rad_ && translation_tolerance_) {
                Eigen::Matrix<double, Dim, Dim> rotation_step =
                    current_state.transform.rotation()
                    * last_state.value().transform.rotation().transpose();

                double angle_diff = 0;
                if constexpr (Dim == icp::Dimension::TwoD) {
                    Eigen::Rotation2Dd rot_step_2d(rotation_step);
                    angle_diff = rot_step_2d.smallestAngle();
                } else {
                    // 3d rotation decompose in 3 param
                    // used Axis_angle to decompose it now: avoiding gimbal lock for 90 degree
                    // rotation that happens in euler angles
                    Eigen::AngleAxisd rot_step_angle_axis(rotation_step);
                    angle_diff = rot_step_angle_axis.angle();
                }

                auto translation = current_state.transform.translation()
                                   - last_state.value().transform.translation();

                if (std::abs(angle_diff) < angle_tolerance_rad_.value()
                    && translation.norm() < translation_tolerance_.value()) {
                    return true;
                }
            }

            return false;
        }

        std::unique_ptr<ICP<Dim>> icp_;

        std::optional<uint64_t> min_iterations_;
        std::optional<uint64_t> max_iterations_;
        std::optional<double> stop_cost_;
        std::optional<double> relative_cost_tolerance_;
        std::optional<double> absolute_cost_tolerance_;
        std::optional<double> angle_tolerance_rad_;
        std::optional<double> translation_tolerance_;
        std::optional<std::chrono::duration<double>> time_limit_;

        std::chrono::time_point<std::chrono::steady_clock> start_time_;
    };
}

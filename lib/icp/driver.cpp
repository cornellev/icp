#include "icp/driver.h"
#include <limits>
#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/AngleAxis.h"
#include "icp/geo.h"
#include "icp/icp.h"
#include "iostream"

namespace icp {
    ICPDriver::ICPDriver(std::unique_ptr<ICP> icp) {
        icp_ = std::move(icp);
    }

    ICPDriver::ConvergenceState ICPDriver::converge(const std::vector<Vector>& a,
        const std::vector<Vector>& b, RBTransform t) {
        start_time_ = std::chrono::steady_clock::now();
        icp_->begin(a, b, t);
        size_t dimension = a[0].size();
        ConvergenceState state(dimension);

        state.iteration_count = 0;
        state.cost = icp_->calculate_cost();
        state.transform = icp_->current_transform();

        std::optional<ConvergenceState> last_state{};

        while (!should_terminate(state, last_state)) {
            icp_->iterate();
            last_state = state;
            state.iteration_count++;
            state.cost = icp_->calculate_cost();
            state.transform = icp_->current_transform();

            std::cout << "Iteration: " << state.iteration_count << std::endl;
            std::cout << "Cost: " << state.cost << std::endl;
            std::cout << "Transform: " << std::endl;
            std::cout << state.transform.rotation << std::endl;
            std::cout << state.transform.translation.transpose() << std::endl;
        }

        return state;
    }

    bool ICPDriver::should_terminate(ConvergenceState current_state,
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
        if (absolute_cost_tolerance_ && std::abs(delta_cost) < absolute_cost_tolerance_.value()) {
            return true;
        }

        double relative_cost_change = std::abs(delta_cost) / current_state.cost;
        if (relative_cost_tolerance_ && relative_cost_change < relative_cost_tolerance_.value()) {
            return true;
        }
        // 3d rotation decompose in 3 param
        // used Axis_angle to decompose it now: avoiding gimbal lock for 90 degree rotation that
        // happens in euler angles
        if (angle_tolerance_rad_ && translation_tolerance_) {
            icp::Matrix rotation_step = current_state.transform.rotation
                                        * last_state.value().transform.rotation.transpose();
            double angle_diff = 0;
            if (icp_->dimensionality() == 2) {
                Eigen::Matrix2d fixed_size_mat(rotation_step);
                Eigen::Rotation2Dd rot_step_2d(fixed_size_mat);
                angle_diff = std::abs(rot_step_2d.smallestAngle());
            } else {
                Eigen::Matrix3d fixed_size_mat(rotation_step);
                Eigen::AngleAxisd rot_step_angle_axis(fixed_size_mat);
                angle_diff = std::abs(rot_step_angle_axis.angle());
            }

            auto translation = current_state.transform.translation
                               - last_state.value().transform.translation;

            std::cout << "Angle difference: " << angle_diff << std::endl;
            std::cout << "Translation difference: " << translation.norm() << std::endl;

            if (angle_diff < angle_tolerance_rad_.value()
                && translation.norm() < translation_tolerance_.value()) {
                return true;
            }
        }

        return false;
    }

    void ICPDriver::set_min_iterations(uint64_t min_iterations) {
        assert(!max_iterations_ || min_iterations <= max_iterations_.value());
        min_iterations_ = min_iterations;
    }

    void ICPDriver::set_max_iterations(uint64_t max_iterations) {
        assert(!min_iterations_ || max_iterations >= min_iterations_.value());
        max_iterations_ = max_iterations;
    }

    void ICPDriver::set_stop_cost(double stop_cost) {
        stop_cost_ = stop_cost;
    }

    void ICPDriver::set_relative_cost_tolerance(double relative_cost_tolerance) {
        relative_cost_tolerance_ = relative_cost_tolerance;
    }

    void ICPDriver::set_absolute_cost_tolerance(double absolute_cost_tolerance) {
        absolute_cost_tolerance_ = absolute_cost_tolerance;
    }

    void ICPDriver::set_transform_tolerance(double angle_tolerance, double translation_tolerance) {
        angle_tolerance_rad_ = angle_tolerance;
        translation_tolerance_ = translation_tolerance;
    }

    void ICPDriver::set_time_limit(std::chrono::duration<double> time_limit) {
        time_limit_ = time_limit;
    }
}

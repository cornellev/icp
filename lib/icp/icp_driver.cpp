#include "icp/icp_driver.h"
#include <limits>

namespace icp {
    ICPDriver::ICPDriver(std::unique_ptr<ICP> icp) {
        icp_ = std::move(icp);
    }

    ICPDriver::ConvergenceState ICPDriver::converge(const std::vector<Vector>& a,
        const std::vector<Vector>& b, RBTransform t) {
        icp_->begin(a, b, t);

        ConvergenceState state{};
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
        }

        return state;
    }

    bool ICPDriver::should_terminate(ConvergenceState current_state,
        std::optional<ConvergenceState> last_state) {
        if (stop_cost_ && current_state.cost < stop_cost_.value()) {
            return true;
        }

        if (min_iterations_ && current_state.iteration_count < min_iterations_.value()) {
            return false;
        }

        if (max_iterations_ && current_state.iteration_count >= max_iterations_.value()) {
            return true;
        }

        if (!last_state) {
            return false;
        }

        double delta_cost = current_state.cost - last_state.value().cost;
        if (delta_cost > 0) {
            // return true;
        }

        if (absolute_cost_tolerance_ && std::abs(delta_cost) < absolute_cost_tolerance_.value()) {
            return true;
        }

        double relative_cost_change = std::abs(delta_cost) / current_state.cost;
        if (relative_cost_tolerance_ && relative_cost_change < relative_cost_tolerance_.value()) {
            return true;
        }

        if (angle_tolerance_rad_ && translation_tolerance_) {
            icp::Vector prev_rot_vector = last_state.value().transform.rotation * icp::Vector(1, 0);
            icp::Vector current_rot_vector = current_state.transform.rotation * icp::Vector(1, 0);
            double dot = prev_rot_vector.dot(current_rot_vector);
            double angle = std::acos(std::clamp(dot, -1.0, 1.0));

            auto translation = current_state.transform.translation
                               - last_state.value().transform.translation;

            if (angle < angle_tolerance_rad_.value()
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
}

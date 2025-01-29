#pragma once

#include <optional>
#include "icp.h"

namespace icp {
    class ICPDriver {
    public:
        /** The result of running `ICPDriver::converge`. */
        struct ConvergenceState {
            /** The cost achieved. */
            double cost;

            /** The number of iterations performed. */
            size_t iteration_count;

            /** The transform. */
            RBTransform transform;
            ConvergenceState(int dim) : transform(dim) {}
        };

        /**
         * @brief Constructs a new ICPDriver using the given ICP method.
         *
         * @param icp The ICP method to use.
         */
        ICPDriver(std::unique_ptr<ICP> icp);

        /**
         * @brief Runs ICP to convergence based on the termination conditions set.
         *
         * @param a The source point cloud.
         * @param b The destination point cloud.
         * @param t The initial guess for the transformation.
         * @return ConvergenceState
         */
        ConvergenceState converge(const std::vector<Vector>& a, const std::vector<Vector>& b,
            RBTransform t);

        /**
         * @brief Sets the minimum number of iterations to run.
         *
         * @param min_iterations The minimum number of iterations to run.
         */
        void set_min_iterations(uint64_t min_iterations);

        /**
         * @brief Sets the maximum number of iterations to run.
         *
         * @param max_iterations The maximum number of iterations to run.
         */
        void set_max_iterations(uint64_t max_iterations);

        /**
         * @brief Sets the cost at which to stop ICP. `converge` will return when a cost below
         * `stop_cost` is achieved.
         *
         * @param stop_cost The cost at which to stop ICP.
         */
        void set_stop_cost(double stop_cost);

        // TODO: fix docs to use math once doxygen is fixed
        /**
         * @brief Sets the relative cost tolerance. `converge` will return when the cost
         * changes by less than this fraction of the current cost, i.e. when |`current_cost` -
         * `prev_cost`| / < `relative_cost_tolerance`.
         *
         * @param relative_cost_tolerance The relative cost tolerance.
         */
        void set_relative_cost_tolerance(double relative_cost_tolerance);

        /**
         * @brief Set the absolute cost tolerance. `converge` will return when the cost changes by
         * less than this amount, i.e. when |`current_cost` - `prev_cost`|  <
         * `absolute_cost_tolerance`.
         *
         * @param absolute_cost_tolerance The absolute cost tolerance.
         */
        void set_absolute_cost_tolerance(double absolute_cost_tolerance);

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
        void set_transform_tolerance(double angle_tolerance, double translation_tolerance);

    private:
        bool should_terminate(ConvergenceState current_state,
            std::optional<ConvergenceState> last_state);

        std::unique_ptr<ICP> icp_;

        int dimension;
        std::optional<uint64_t> min_iterations_;
        std::optional<uint64_t> max_iterations_;
        std::optional<double> stop_cost_;
        std::optional<double> relative_cost_tolerance_;
        std::optional<double> absolute_cost_tolerance_;
        std::optional<double> angle_tolerance_rad_;
        std::optional<double> translation_tolerance_;
    };
}

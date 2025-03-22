#pragma once

#include <memory>
#include <Eigen/Dense>
#include "icp/icp.h"
#include "algo/kdtree.h"

namespace icp {

    struct NEIGHBOR {
        std::vector<float> distances;
        std::vector<size_t> indices;
    };

    class Vanilla_3d : public ICP {
    public:
        Vanilla_3d(const Config& config);
        Vanilla_3d();

        void set_target(const std::vector<Vector>& target) override;
        double get_current_cost() const;

    protected:
        void setup() override;
        void iterate() override;

    private:
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd C;
        std::unique_ptr<KdTree<Vector>> target_kdtree_;
        double current_cost_;

        void rebuild_kdtree();
        NEIGHBOR nearest_neighbor(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);
        float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);
        Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
        void calculate_cost(const std::vector<float>& distances);
    };

}  // namespace icp
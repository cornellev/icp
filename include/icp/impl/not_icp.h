#pragma once

#include <memory>
#include <Eigen/Dense>
#include "icp/icp.h"
#include "icp/config.h"
#include "algo/kdtree.h"

namespace icp {

    class Not_icp : public ICP3 {
    public:
        Not_icp(const Config& config);
        Not_icp();
        ~Not_icp();

    protected:
        void setup() override;
        void iterate() override;

    private:
        PointCloud c;
        std::unique_ptr<KdTree<Vector>> target_kdtree_;
        double current_cost_;
        Eigen::Matrix3d initial_rotation;

        void rebuild_kdtree();
        NEIGHBOR nearest_neighbor(const PointCloud& src, const PointCloud& dst);
        float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);
        RBTransform best_fit_transform(const PointCloud& A, const PointCloud& B);
        void calculate_cost(const std::vector<float>& distances);
    };

}  // namespace icp
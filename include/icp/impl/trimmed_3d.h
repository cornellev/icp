#pragma once

#include <memory>
#include <Eigen/Dense>
#include "icp/icp.h"
#include "icp/config.h"
#include "algo/kdtree.h"

namespace icp {

    class Trimmed_3d : public ICP3 {
    public:
        Trimmed_3d(const Config& config);
        Trimmed_3d();
        ~Trimmed_3d();

    protected:
        void setup() override;
        void iterate() override;

    private:
        PointCloud c;
        std::unique_ptr<KdTree<Vector>> target_kdtree_;
        double current_cost_;
        double max_distance;  // max distance for trimming

        NEIGHBOR nearest_neighbor(const PointCloud& src, const PointCloud& dst);
        float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);
        RBTransform best_fit_transform(const PointCloud& A, const PointCloud& B);
        void calculate_cost(const std::vector<float>& distances);
    };

}  // namespace icp
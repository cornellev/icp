// TODO: we should probably find a good solution for CEV copyright

/*
 * @author Utku Melemetci
 */

#pragma once

#include "icp/icp.h"
#include "algo/kdtree.h"
#include "icp/config.h"

namespace icp {
    class Vanilla final : public ICP2 {
    public:
        Vanilla();
        Vanilla(const Config& config);
        ~Vanilla();

        void setup() override;
        void iterate() override;

    private:
        void rebuild_kdtree();
        void compute_matches();
        void compute_match_for_point(size_t i);

        std::unique_ptr<KdTree<Vector>> target_kdtree_;

        PointCloud a_current;
    };
}
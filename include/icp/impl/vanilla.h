// TODO: we should probably find a good solution for CEV copyright

/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"
#include "algo/kdtree.h"

namespace icp {
    class Vanilla final : public ICP {
    public:
        Vanilla();
        Vanilla(const Config& config);
        ~Vanilla();

        void setup() override;
        void iterate() override;

        void set_target(const std::vector<Vector>& target) override;

    private:
        void rebuild_kdtree();
        void compute_matches();
        void compute_match_for_point(size_t i);

        std::unique_ptr<KdTree<Vector>> target_kdtree_;

    protected:
        std::vector<icp::Vector> a_current;
    };
}
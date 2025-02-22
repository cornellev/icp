#include "icp/icp.h"

typedef struct {
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

namespace icp {
    class Vanilla_3d final : public ICP {
    public:
        Vanilla_3d();
        Vanilla_3d(const Config& config);
        ~Vanilla_3d();

        void setup() override;
        void iterate() override;

    private:
        NEIGHBOR nearest_neighbor(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);
        float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);
        Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
        Eigen::MatrixXd C;  // the current source point cloud
    };
}
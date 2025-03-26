/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"
#include "icp/config.h"

namespace icp {
    class Trimmed final : public ICP2d {
    public:
        Trimmed(double overlap_rate);
        Trimmed(const Config& config);
        ~Trimmed();

        void setup() override;
        void iterate() override;

    private:
        void compute_matches();

        double overlap_rate;
        PointCloud a_current;
    };
}

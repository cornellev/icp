/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"

namespace icp {
    class Trimmed final : public ICP {
    public:
        Trimmed(double overlap_rate);
        Trimmed(const Config& config);
        ~Trimmed();

        void setup() override;
        void iterate() override;

    private:
        double overlap_rate;
        std::vector<icp::Vector> a_current;
        icp::Vector b_cm;
    };
}

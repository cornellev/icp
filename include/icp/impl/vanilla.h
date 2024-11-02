// TODO: we should probably find a good solution for CEV copyright

/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"

namespace icp {
    class Vanilla final : public ICP {
    public:
        Vanilla();
        Vanilla(const Config& config);
        ~Vanilla();

        void setup() override;
        void iterate() override;

    private:
        std::vector<icp::Vector> a_current;
        icp::Vector b_cm;
    };
}
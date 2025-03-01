/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"

namespace icp {
    class Trimmed : public ICP {
    public:
        Trimmed(double overlap_rate);
        Trimmed(const Config& config);
        ~Trimmed();

        void setup() override;
        void iterate() override;
        RBTransform get_last_step() const override {
            return last_step;
        }
        const std::vector<Match>& get_next_matches() const override {
            return next_matches;
        }

    protected:
        RBTransform last_step;
        std::vector<Match> next_matches;

    private:
        void compute_matches();

        double overlap_rate;
        std::vector<icp::Vector> a_current;
        icp::Vector b_cm;
    };
}

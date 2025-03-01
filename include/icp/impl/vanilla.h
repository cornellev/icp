// TODO: we should probably find a good solution for CEV copyright

/*
 * @author Utku Melemetci
 */

#include "icp/icp.h"

namespace icp {
    class Vanilla : public ICP {
    protected:
        std::vector<Vector> a_current;
        std::vector<Vector> a_next;
        std::vector<Match> next_matches;
        std::vector<Match> matches;
        RBTransform last_step = RBTransform();

    public:
        Vanilla();
        Vanilla(const Config& config);
        ~Vanilla();

        void setup() override;
        void iterate() override;

        const std::vector<Match>& get_next_matches() const override {
            return next_matches;
        }
        RBTransform get_last_step() const override {
            return last_step;
        }

    private:
        // void compute_matches();
        void compute_matches(const std::vector<Vector>& source, std::vector<Match>& results);

        // std::vector<icp::Vector> a_current;
        //  icp::Vector a_current_cm;
        //  icp::Vector corr_cm = icp::Vector::Zero();
        //  size_t n;
    };
}
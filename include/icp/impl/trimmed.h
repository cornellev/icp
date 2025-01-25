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

        //get active match count
        //size_t get_active_match_count() const { return active_match_count; }

    private:
        void compute_matches();

        std::vector<Match> forward_matches;  // A到B的匹配
        std::vector<Match> backward_matches; // B到A的匹配

        double overlap_rate;
        std::vector<icp::Vector> a_current;
        icp::Vector b_cm;
        //size_t active_match_count;
    };
}

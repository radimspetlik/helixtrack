#pragma once

#include <string>
#include <vector>

#include "config.h"

class GroundTruthRpm {
public:
    void clear();
    void set_samples(std::vector<long long> pol1_timestamps_us, std::vector<double> rpm_values);
    double at_us(long long ts_us) const;

private:
    std::vector<long long> pol1_timestamps_us_;
    std::vector<double> rpm_at_pol1_;
};

void load_start_and_end_us_from_rpm_alignment_if_available(
    Config &config, int channel_num, GroundTruthRpm &ground_truth_rpm);
void load_tracker_init_if_available(Config &config, int channel_num);

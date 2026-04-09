#pragma once

#include <string>
#include <vector>

struct PlaybackPlanStep {
    long long rel_us = 0;
    double fps = 0.0;
    long long agg_us = 0;
    long long abs_us = -1;
    bool applied = false;
};

bool load_playback_plan_file(const std::string &path, std::vector<PlaybackPlanStep> &out);

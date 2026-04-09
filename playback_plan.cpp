#include "playback_plan.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

bool load_playback_plan_file(const std::string &path, std::vector<PlaybackPlanStep> &out) {
    out.clear();
    if (path.empty()) return false;

    std::ifstream input(path);
    if (!input) {
        std::cerr << "Playback plan: cannot open '" << path << "'. Ignoring.\n";
        return false;
    }

    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;

        const auto not_ws = line.find_first_not_of(" \t\r\n");
        if (not_ws == std::string::npos) continue;
        if (line.compare(not_ws, 1, "#") == 0 || line.compare(not_ws, 2, "//") == 0) continue;

        std::istringstream stream(line);
        std::string rel_token;
        std::string fps_token;
        std::string agg_token;
        if (!std::getline(stream, rel_token, ',')) continue;
        if (!std::getline(stream, fps_token, ',')) continue;
        if (!std::getline(stream, agg_token, ',')) {
            if (rel_token.empty() || fps_token.empty()) continue;
        }

        try {
            PlaybackPlanStep step;
            step.rel_us = std::stoll(rel_token);
            step.fps = std::stod(fps_token);
            step.agg_us = std::stoll(agg_token);
            out.push_back(step);
        } catch (...) {
            std::cerr << "Playback plan: parse error at line " << line_number << " -> '" << line << "'\n";
        }
    }

    std::sort(out.begin(), out.end(), [](const PlaybackPlanStep &a, const PlaybackPlanStep &b) {
        return a.rel_us < b.rel_us;
    });
    return !out.empty();
}

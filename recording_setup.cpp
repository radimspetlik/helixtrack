#include "recording_setup.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

void GroundTruthRpm::clear() {
    pol1_timestamps_us_.clear();
    rpm_at_pol1_.clear();
}

void GroundTruthRpm::set_samples(std::vector<long long> pol1_timestamps_us, std::vector<double> rpm_values) {
    pol1_timestamps_us_ = std::move(pol1_timestamps_us);
    rpm_at_pol1_ = std::move(rpm_values);
}

double GroundTruthRpm::at_us(long long ts_us) const {
    if (pol1_timestamps_us_.size() < 2) return std::numeric_limits<double>::quiet_NaN();
    auto it = std::upper_bound(pol1_timestamps_us_.begin(), pol1_timestamps_us_.end(), ts_us);
    if (it == pol1_timestamps_us_.begin()) return std::numeric_limits<double>::quiet_NaN();
    std::size_t index = static_cast<std::size_t>(it - pol1_timestamps_us_.begin()) - 1;
    if (index == 0 && rpm_at_pol1_.size() > 1) index = 1;
    return index < rpm_at_pol1_.size() ? rpm_at_pol1_[index] : rpm_at_pol1_.back();
}

void load_start_and_end_us_from_rpm_alignment_if_available(
    Config &config, int channel_num, GroundTruthRpm &ground_truth_rpm) {
    const std::string recording_dir = std::filesystem::path(config.recording_filepath).parent_path().string();
    const std::string recording_filename = std::filesystem::path(config.recording_filepath).filename().string();
    const std::string recording_filename_no_ext =
        recording_filename.substr(0, recording_filename.find_last_of('.'));
    const std::string rpm_alignment_dir = recording_dir + "/rpm_alignment";
    const std::string rpm_alignment_path = rpm_alignment_dir + "/" + recording_filename_no_ext + ".csv";

    std::cout << rpm_alignment_path << std::endl;
    if (!std::filesystem::exists(rpm_alignment_path)) return;

    std::ifstream rpm_alignment_file(rpm_alignment_path);
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(rpm_alignment_file, line)) {
        lines.push_back(line);
    }

    if (lines.size() >= 1000) {
        if (config.start_us == 0) {
            std::istringstream stream(lines[1000]);
            std::string token;
            std::getline(stream, token, ',');
            std::getline(stream, token, ',');
            std::getline(stream, token, ',');
            try {
                config.start_us = std::stoll(token);
            } catch (...) {
            }
        }
        if (config.end_us == 0) {
            std::istringstream stream(lines[lines.size() - 1000]);
            std::string token;
            std::getline(stream, token, ',');
            std::getline(stream, token, ',');
            std::getline(stream, token, ',');
            try {
                config.end_us = std::stoll(token);
            } catch (...) {
            }
        }
        std::cout << "Loaded from rpm_alignment csv start_us: " << config.start_us
                  << ", end_us: " << config.end_us << std::endl;
    }

    std::vector<long long> pol1_timestamps_us;
    std::vector<double> rpm_at_pol1;
    long long last_us = 0;
    for (std::size_t i = 1; i < lines.size(); ++i) {
        std::istringstream stream(lines[i]);
        std::string c0, c1, c2, c3, c4, c5, c6;
        if (!std::getline(stream, c0, ',')) continue;
        if (!std::getline(stream, c1, ',')) continue;
        if (!std::getline(stream, c2, ',')) continue;
        if (!std::getline(stream, c3, ',')) continue;
        if (!std::getline(stream, c4, ',')) continue;
        if (!std::getline(stream, c5, ',')) c5.clear();
        if (!std::getline(stream, c6, ',')) c6.clear();

        try {
            long long us = std::stoll(c2);
            const int polarity = std::stoi(c4);
            const int channel = std::stoi(c3);
            if (channel != channel_num || polarity != 1) continue;

            if (!c6.empty()) {
                const double rpm = std::stod(c6);
                if (rpm > 0.0 && last_us > 0) {
                    const long long dt_us = static_cast<long long>(60000000.0 / rpm);
                    us = last_us + dt_us;
                }
                last_us = us;
            }

            pol1_timestamps_us.push_back(us);
        } catch (...) {
            continue;
        }
    }

    if (pol1_timestamps_us.size() >= 2) {
        rpm_at_pol1.resize(pol1_timestamps_us.size());
        rpm_at_pol1[0] = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 1; i < pol1_timestamps_us.size(); ++i) {
            const long long dt_us = pol1_timestamps_us[i] - pol1_timestamps_us[i - 1];
            if (dt_us > 0) {
                rpm_at_pol1[i] = 120000000.0 / static_cast<double>(dt_us);
            } else {
                rpm_at_pol1[i] = std::numeric_limits<double>::quiet_NaN();
            }
        }

        const auto loaded_count = pol1_timestamps_us.size();
        const auto first_rpm = rpm_at_pol1.front();
        ground_truth_rpm.set_samples(std::move(pol1_timestamps_us), std::move(rpm_at_pol1));
        std::cout << "GT RPM: loaded " << loaded_count << " first RPM GT:" << first_rpm
                  << "RPM pol==1 timestamps from rpm_alignment." << std::endl;
    } else {
        ground_truth_rpm.clear();
        std::cout << "GT RPM: insufficient pol==1 timestamps in rpm_alignment." << std::endl;
    }
}

void load_tracker_init_if_available(Config &config, int channel_num) {
    const std::string recording_dir = std::filesystem::path(config.recording_filepath).parent_path().string();
    const std::string recording_filename = std::filesystem::path(config.recording_filepath).filename().string();
    const std::string recording_filename_no_ext =
        recording_filename.substr(0, recording_filename.find_last_of('.'));
    const std::string tracker_dir = recording_dir + "/tracker_initial_positions";
    const std::string tracker_path = tracker_dir + "/" + recording_filename_no_ext + ".csv";

    if (!std::filesystem::exists(tracker_path)) return;

    std::ifstream tracker_initial_positions_file(tracker_path);
    std::string line;
    std::getline(tracker_initial_positions_file, line);
    int current_channel_num = 0;
    while (std::getline(tracker_initial_positions_file, line)) {
        std::string token;
        std::istringstream stream(line);
        std::getline(stream, token, ',');
        std::getline(stream, token, ',');
        std::getline(stream, token, ',');
        config.tracker_init_x_center = std::stoi(token);
        std::getline(stream, token, ',');
        config.tracker_init_y_center = std::stoi(token);
        std::getline(stream, token, ',');
        config.tracker_rotation_sign = std::stoi(token);
        std::getline(stream, token, ',');
        config.tracker_init_aspect_ratio = std::stof(token);
        std::getline(stream, token, ',');
        config.tracker_init_radius_px = std::stof(token);
        std::getline(stream, token, ',');
        config.tracker_init_psi = std::stof(token);
        if (current_channel_num == channel_num) {
            break;
        }
        ++current_channel_num;
    }
}

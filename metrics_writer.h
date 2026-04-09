#pragma once

#include <vector>

#include "config.h"

struct RunMetricsConfig {
    long long sample_dt_us;
    double noise_window_s;
    double steady_gt_slope_rpm_s;
    double step_threshold_rpm;
    double settling_tol_pct;
    double ramp_min_rpm_per_s;
};

struct RunMetricsData {
    const std::vector<long long> &sample_ts_us;
    const std::vector<long long> &sample_dt_us_vec;
    const std::vector<long long> &latency_us_vec;
    const std::vector<double> &rpm_est_vec;
    const std::vector<double> &rpm_gt_vec;
    const std::vector<double> &err_rpm_vec;
    const std::vector<double> &samp_u_cx;
    const std::vector<double> &samp_u_cy;
    const std::vector<double> &H11_vec;
    const std::vector<double> &H12_vec;
    const std::vector<double> &H13_vec;
    const std::vector<double> &H21_vec;
    const std::vector<double> &H22_vec;
    const std::vector<double> &H23_vec;
    const std::vector<double> &H31_vec;
    const std::vector<double> &H32_vec;
    const std::vector<double> &H33_vec;
    const std::vector<long long> &per_event_us_samples;
    const std::vector<long long> &solve_us;
    const std::vector<long long> &solve_batch_sizes;
    long long total_expected_samples;
};

void write_run_metrics(const Config &config, const RunMetricsConfig &metrics_config,
                       const RunMetricsData &metrics_data);

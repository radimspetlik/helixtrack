#include "metrics_writer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "H5Cpp.h"

#include "hdf_utils.h"
#include "stats_utils.h"

namespace {

double median_range(const std::vector<double> &values, int begin, int end) {
    begin = std::max(begin, 0);
    end = std::min(end, static_cast<int>(values.size()) - 1);
    std::vector<double> slice;
    slice.reserve(std::max(0, end - begin + 1));
    for (int i = begin; i <= end; ++i) {
        if (std::isfinite(values[i])) slice.push_back(values[i]);
    }
    if (slice.empty()) return NAN;
    return percentile(slice, 0.5);
}

}  // namespace

void write_run_metrics(const Config &config, const RunMetricsConfig &metrics_config,
                       const RunMetricsData &metrics_data) {
    const bool have_gt = std::any_of(metrics_data.rpm_gt_vec.begin(), metrics_data.rpm_gt_vec.end(),
                                     [](double value) { return std::isfinite(value); });

    const std::string analysis_dir = std::filesystem::path(config.analysis_filepath).parent_path().string();
    std::error_code ec;
    std::filesystem::create_directories(analysis_dir, ec);
    const std::string h5_out = config.analysis_filepath;

    H5::H5File out(h5_out, H5F_ACC_TRUNC);
    auto channels = out.createGroup("/channels");
    std::string ch_name = "ch" + std::to_string(config.channel_num);
    auto chg = channels.createGroup(ch_name);
    auto g_samples = chg.createGroup("samples");
    auto g_stats = chg.createGroup("stats");
    auto g_dyn = chg.createGroup("dynamic");
    auto g_steps = g_dyn.createGroup("steps");
    auto g_lat = chg.createGroup("latency");
    auto g_comp = chg.createGroup("compute");

    {
        std::string n = "ts_us";
        write_results_to_hdf5(g_samples, n, metrics_data.sample_ts_us);
    }
    {
        std::string n = "rpm_est";
        write_results_to_hdf5(g_samples, n, metrics_data.rpm_est_vec);
    }
    {
        std::string n = "rpm_gt";
        write_results_to_hdf5(g_samples, n, metrics_data.rpm_gt_vec);
    }
    {
        std::vector<double> aligned_error;
        aligned_error.reserve(metrics_data.rpm_est_vec.size());
        for (std::size_t i = 0; i < metrics_data.rpm_est_vec.size(); ++i) {
            const double gt = metrics_data.rpm_gt_vec[i];
            if (std::isfinite(gt)) aligned_error.push_back(metrics_data.rpm_est_vec[i] - gt);
            else aligned_error.push_back(std::numeric_limits<double>::quiet_NaN());
        }
        std::string n = "error_rpm";
        write_results_to_hdf5(g_samples, n, aligned_error);
    }
    {
        std::string n = "latency_us";
        write_results_to_hdf5(g_samples, n, metrics_data.latency_us_vec);
    }

    if (!metrics_data.samp_u_cx.empty()) {
        std::string n = "u_cx";
        write_results_to_hdf5(g_samples, n, metrics_data.samp_u_cx);
        n = "u_cy";
        write_results_to_hdf5(g_samples, n, metrics_data.samp_u_cy);
    }
    if (!metrics_data.H11_vec.empty()) {
        std::string n = "H11";
        write_results_to_hdf5(g_samples, n, metrics_data.H11_vec);
        n = "H12";
        write_results_to_hdf5(g_samples, n, metrics_data.H12_vec);
        n = "H13";
        write_results_to_hdf5(g_samples, n, metrics_data.H13_vec);
        n = "H21";
        write_results_to_hdf5(g_samples, n, metrics_data.H21_vec);
        n = "H22";
        write_results_to_hdf5(g_samples, n, metrics_data.H22_vec);
        n = "H23";
        write_results_to_hdf5(g_samples, n, metrics_data.H23_vec);
        n = "H31";
        write_results_to_hdf5(g_samples, n, metrics_data.H31_vec);
        n = "H32";
        write_results_to_hdf5(g_samples, n, metrics_data.H32_vec);
        n = "H33";
        write_results_to_hdf5(g_samples, n, metrics_data.H33_vec);
    }

    double est_min = metrics_data.rpm_est_vec.empty() ? NAN
                                                      : *std::min_element(metrics_data.rpm_est_vec.begin(),
                                                                          metrics_data.rpm_est_vec.end());
    double est_max = metrics_data.rpm_est_vec.empty() ? NAN
                                                      : *std::max_element(metrics_data.rpm_est_vec.begin(),
                                                                          metrics_data.rpm_est_vec.end());
    double gt_min = NAN;
    double gt_max = NAN;
    if (have_gt) {
        std::vector<double> gt_copy;
        gt_copy.reserve(metrics_data.rpm_gt_vec.size());
        for (double value : metrics_data.rpm_gt_vec) {
            if (std::isfinite(value)) gt_copy.push_back(value);
        }
        if (!gt_copy.empty()) {
            gt_min = *std::min_element(gt_copy.begin(), gt_copy.end());
            gt_max = *std::max_element(gt_copy.begin(), gt_copy.end());
        }
    }
    write_scalar_1d(g_stats, "est_rpm_min", est_min);
    write_scalar_1d(g_stats, "est_rpm_max", est_max);
    write_scalar_1d(g_stats, "gt_rpm_min", gt_min);
    write_scalar_1d(g_stats, "gt_rpm_max", gt_max);

    const double mean_dt_us = mean_of(metrics_data.sample_dt_us_vec);
    const double jitter_us = std_of(metrics_data.sample_dt_us_vec, mean_dt_us);
    const double update_rate_hz = (mean_dt_us > 0) ? (1e6 / mean_dt_us) : 0.0;
    const long long longest_gap_us = metrics_data.sample_dt_us_vec.empty()
                                         ? 0
                                         : *std::max_element(metrics_data.sample_dt_us_vec.begin(),
                                                             metrics_data.sample_dt_us_vec.end());
    double dropout_rate = 0.0;
    if (metrics_data.total_expected_samples > 0) {
        const long long actual = static_cast<long long>(metrics_data.sample_ts_us.size());
        const long long missing = std::max<long long>(0, metrics_data.total_expected_samples - actual);
        dropout_rate = double(missing) / double(metrics_data.total_expected_samples);
    }
    write_scalar_1d(g_stats, "update_rate_hz", update_rate_hz);
    write_scalar_1d(g_stats, "sample_dt_us_mean", mean_dt_us);
    write_scalar_1d(g_stats, "sampling_jitter_us", jitter_us);
    write_scalar_1d(g_stats, "dropouts_rate", dropout_rate);
    write_scalar_1d(g_stats, "longest_gap_us", longest_gap_us);
    write_scalar_1d(g_stats, "total_expected_samples", metrics_data.total_expected_samples);

    const long long p50_lat = metrics_data.latency_us_vec.empty() ? 0 : percentile(metrics_data.latency_us_vec, 0.50);
    const long long p95_lat = metrics_data.latency_us_vec.empty() ? 0 : percentile(metrics_data.latency_us_vec, 0.95);
    write_scalar_1d(g_lat, "processing_latency_us_p50", p50_lat);
    write_scalar_1d(g_lat, "processing_latency_us_p95", p95_lat);

    if (have_gt && !metrics_data.err_rpm_vec.empty()) {
        const double bias = mean_of(metrics_data.err_rpm_vec);
        long double mae_sum = 0;
        long double rmse_sum = 0;
        for (double err : metrics_data.err_rpm_vec) {
            mae_sum += std::abs(err);
            rmse_sum += err * err;
        }
        const double mae = double(mae_sum / metrics_data.err_rpm_vec.size());
        const double rmse = std::sqrt(double(rmse_sum / metrics_data.err_rpm_vec.size()));
        write_scalar_1d(g_stats, "bias_rpm", bias);
        write_scalar_1d(g_stats, "mae_rpm", mae);
        write_scalar_1d(g_stats, "rmse_rpm", rmse);

        const int window = std::max(
            1, int(std::round(metrics_config.noise_window_s * 1e6 /
                              std::max<long long>(1, metrics_config.sample_dt_us))));
        std::vector<double> noise_std;
        if (int(metrics_data.sample_ts_us.size()) >= (window + 1)) {
            for (int i = 0; i + window < int(metrics_data.sample_ts_us.size()); ++i) {
                if (!std::isfinite(metrics_data.rpm_gt_vec[i]) ||
                    !std::isfinite(metrics_data.rpm_gt_vec[i + window])) {
                    continue;
                }
                const double dgt =
                    (metrics_data.rpm_gt_vec[i + window] - metrics_data.rpm_gt_vec[i]) /
                    ((metrics_data.sample_ts_us[i + window] - metrics_data.sample_ts_us[i]) * 1e-6);
                if (std::abs(dgt) <= metrics_config.steady_gt_slope_rpm_s) {
                    std::vector<double> win;
                    win.reserve(window + 1);
                    for (int k = 0; k <= window; ++k) {
                        const double gt = metrics_data.rpm_gt_vec[i + k];
                        if (std::isfinite(gt)) win.push_back(metrics_data.rpm_est_vec[i + k] - gt);
                    }
                    if (win.size() >= 2) {
                        const double mu = mean_of(win);
                        noise_std.push_back(std_of(win, mu));
                    }
                }
            }
        }
        const double noise_mean = noise_std.empty() ? NAN : mean_of(noise_std);
        const double noise_p95 = noise_std.empty() ? NAN : percentile(noise_std, 0.95);
        write_scalar_1d(g_stats, "noise_std_rpm_mean", noise_mean);
        write_scalar_1d(g_stats, "noise_std_rpm_p95", noise_p95);
    }

    double min_step = NAN;
    double gran_ratio = NAN;
    if (metrics_data.rpm_est_vec.size() >= 2) {
        constexpr double kQuantizeRpm = 0.01;
        double prev = round_to(metrics_data.rpm_est_vec[0], kQuantizeRpm);
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 1; i < metrics_data.rpm_est_vec.size(); ++i) {
            const double cur = round_to(metrics_data.rpm_est_vec[i], kQuantizeRpm);
            const double diff = std::abs(cur - prev);
            if (diff > 0 && diff < best) best = diff;
            prev = cur;
        }
        if (std::isfinite(best)) min_step = best;
        const double span = (std::isfinite(est_min) && std::isfinite(est_max)) ? (est_max - est_min) : NAN;
        if (std::isfinite(span) && span > 0 && std::isfinite(min_step)) gran_ratio = min_step / span;
    }
    write_scalar_1d(g_stats, "resolution_min_step_rpm", min_step);
    write_scalar_1d(g_stats, "granularity_ratio", gran_ratio);

    double ramp_mae = NAN;
    if (have_gt && metrics_data.sample_ts_us.size() >= 2) {
        std::vector<double> ramp_errors;
        for (std::size_t i = 1; i < metrics_data.sample_ts_us.size(); ++i) {
            if (!std::isfinite(metrics_data.rpm_gt_vec[i]) || !std::isfinite(metrics_data.rpm_gt_vec[i - 1])) continue;
            const double dt_s = (metrics_data.sample_ts_us[i] - metrics_data.sample_ts_us[i - 1]) * 1e-6;
            if (dt_s <= 0) continue;
            const double slope = (metrics_data.rpm_gt_vec[i] - metrics_data.rpm_gt_vec[i - 1]) / dt_s;
            if (std::abs(slope) >= metrics_config.ramp_min_rpm_per_s &&
                std::isfinite(metrics_data.rpm_gt_vec[i])) {
                ramp_errors.push_back(std::abs(metrics_data.rpm_est_vec[i] - metrics_data.rpm_gt_vec[i]));
            }
        }
        if (!ramp_errors.empty()) {
            long double sum = 0;
            for (double err : ramp_errors) sum += err;
            ramp_mae = double(sum / ramp_errors.size());
        }
    }
    write_scalar_1d(g_stats, "ramp_mae_rpm", ramp_mae);

    if (have_gt && metrics_data.sample_ts_us.size() >= 3) {
        std::vector<int> step_idx;
        for (std::size_t i = 1; i < metrics_data.rpm_gt_vec.size(); ++i) {
            if (!std::isfinite(metrics_data.rpm_gt_vec[i]) || !std::isfinite(metrics_data.rpm_gt_vec[i - 1])) continue;
            if (std::abs(metrics_data.rpm_gt_vec[i] - metrics_data.rpm_gt_vec[i - 1]) >=
                metrics_config.step_threshold_rpm) {
                step_idx.push_back(int(i));
            }
        }

        std::vector<long long> step_ts;
        std::vector<long long> rise_us;
        std::vector<long long> settling_us;
        std::vector<double> overshoot_pc;
        std::vector<double> step_amp;
        for (int idx : step_idx) {
            const int win = std::max(1, int(std::round(0.2 * 1e6 / metrics_config.sample_dt_us)));
            const double x0 = median_range(metrics_data.rpm_gt_vec, idx - win, idx - 1);
            const double x1 = median_range(metrics_data.rpm_gt_vec, idx + 1, idx + win);
            if (!std::isfinite(x0) || !std::isfinite(x1)) continue;
            const double amplitude = x1 - x0;
            if (std::abs(amplitude) < 1e-6) continue;

            const double y10 = x0 + 0.1 * amplitude;
            const double y90 = x0 + 0.9 * amplitude;
            auto cross_time = [&](double level) -> long long {
                for (int k = std::max(1, idx - win);
                     k < std::min(int(metrics_data.rpm_est_vec.size()), idx + 5 * win); ++k) {
                    const double y0v = metrics_data.rpm_est_vec[k - 1];
                    const double y1v = metrics_data.rpm_est_vec[k];
                    if ((y0v - level) * (y1v - level) <= 0) {
                        const long long t0 = metrics_data.sample_ts_us[k - 1];
                        const long long t1 = metrics_data.sample_ts_us[k];
                        const double alpha = (std::abs(y1v - y0v) < 1e-9) ? 0.0 : ((level - y0v) / (y1v - y0v));
                        return static_cast<long long>(t0 + std::clamp(alpha, 0.0, 1.0) * (t1 - t0));
                    }
                }
                return 0;
            };

            const long long t10 = cross_time(y10);
            const long long t90 = cross_time(y90);
            if (t10 == 0 || t90 == 0) continue;

            const long long t_step = metrics_data.sample_ts_us[idx];
            const long long trise = t90 - t10;

            double ypeak = x0;
            for (int k = idx; k < std::min(int(metrics_data.rpm_est_vec.size()), idx + 10 * win); ++k) {
                ypeak = std::max(ypeak, metrics_data.rpm_est_vec[k]);
            }
            const double overs = (amplitude > 0)
                                     ? ((ypeak - x1) / std::abs(amplitude)) * 100.0
                                     : ((x1 - (*std::min_element(metrics_data.rpm_est_vec.begin() + idx,
                                                                 metrics_data.rpm_est_vec.begin() +
                                                                     std::min(int(metrics_data.rpm_est_vec.size()),
                                                                              idx + 10 * win)))) /
                                        std::abs(amplitude)) *
                                           100.0;

            const double band = (std::abs(x1) > 1e-6)
                                    ? (metrics_config.settling_tol_pct * 0.01 * std::abs(x1))
                                    : (metrics_config.settling_tol_pct * 0.01 * std::abs(amplitude));
            const int need = std::max(2, int(std::round(0.1 * 1e6 / metrics_config.sample_dt_us)));
            long long t_set = 0;
            int streak = 0;
            for (int k = idx; k < int(metrics_data.rpm_est_vec.size()); ++k) {
                if (std::abs(metrics_data.rpm_est_vec[k] - x1) <= band) {
                    if (++streak >= need) {
                        t_set = metrics_data.sample_ts_us[k];
                        break;
                    }
                } else {
                    streak = 0;
                }
            }

            step_ts.push_back(t_step);
            rise_us.push_back(trise);
            overshoot_pc.push_back(overs);
            settling_us.push_back(t_set ? (t_set - t_step) : 0);
            step_amp.push_back(amplitude);
        }

        if (!step_ts.empty()) {
            std::string n = "step_ts_us";
            write_results_to_hdf5(g_steps, n, step_ts);
            n = "rise_time_us_10_90";
            write_results_to_hdf5(g_steps, n, rise_us);
            n = "overshoot_pct";
            write_results_to_hdf5(g_steps, n, overshoot_pc);
            n = "settling_time_us_2pct";
            write_results_to_hdf5(g_steps, n, settling_us);
            n = "step_amplitude_rpm";
            write_results_to_hdf5(g_steps, n, step_amp);
        }
    }

    double spike_rate = NAN;
    if (metrics_data.rpm_est_vec.size() >= 3) {
        std::vector<double> delta_rpm;
        delta_rpm.reserve(metrics_data.rpm_est_vec.size() - 1);
        for (std::size_t i = 1; i < metrics_data.rpm_est_vec.size(); ++i) {
            delta_rpm.push_back(metrics_data.rpm_est_vec[i] - metrics_data.rpm_est_vec[i - 1]);
        }
        std::vector<double> absd;
        absd.reserve(delta_rpm.size());
        const double median_d = percentile(delta_rpm, 0.5);
        for (double value : delta_rpm) absd.push_back(std::abs(value - median_d));
        const double mad = percentile(absd, 0.5);
        const double threshold = 6.0 * (mad > 0 ? mad : std_of(delta_rpm, mean_of(delta_rpm)));
        long long spikes = 0;
        for (double value : delta_rpm) {
            if (std::abs(value - median_d) > threshold) ++spikes;
        }
        spike_rate = delta_rpm.empty() ? NAN : double(spikes) / double(delta_rpm.size());
    }
    write_scalar_1d(g_stats, "spike_rate", spike_rate);

    if (!metrics_data.per_event_us_samples.empty()) {
        const long long p95 = percentile(metrics_data.per_event_us_samples, 0.95);
        const long long max_value =
            *std::max_element(metrics_data.per_event_us_samples.begin(), metrics_data.per_event_us_samples.end());
        std::string n = "per_event_update_us_samples";
        write_results_to_hdf5(g_comp, n, metrics_data.per_event_us_samples);
        write_scalar_1d(g_comp, "per_event_update_us_p95", p95);
        write_scalar_1d(g_comp, "per_event_update_us_max", max_value);
    }
    if (!metrics_data.solve_us.empty()) {
        const long long p95 = percentile(metrics_data.solve_us, 0.95);
        const long long max_value =
            *std::max_element(metrics_data.solve_us.begin(), metrics_data.solve_us.end());
        std::string n = "solve_us";
        write_results_to_hdf5(g_comp, n, metrics_data.solve_us);
        n = "solve_batch_size";
        write_results_to_hdf5(g_comp, n, metrics_data.solve_batch_sizes);
        write_scalar_1d(g_comp, "solve_us_p95", p95);
        write_scalar_1d(g_comp, "solve_us_max", max_value);
    }

    std::cout << "Metrics written to " << h5_out << std::endl;
}

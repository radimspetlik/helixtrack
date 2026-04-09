#include <iostream>
#include <vector>
#include <tuple>
#include <string>
#include <filesystem>
#include <fstream>
#include <functional>
#include <thread>
#include <algorithm>
#include <atomic>
#include <sstream>
#include <limits>
#include <cmath>
#include <cstdlib>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>
#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/event_frame_histo_generation_algorithm.h>
#include <metavision/hal/facilities/i_hw_identification.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <numeric>
#include <set>
#include <array>

#include "config.h"
#include "const.h"
#include "metrics_writer.h"
#include "helixtrack.h"
#include "playback_plan.h"
#include "recording_setup.h"

using Clock = std::chrono::steady_clock;

int main(int argc, char *argv[]) {
    std::string config_filepath = "config.yaml";
    if (argc >= 2) {
        config_filepath = argv[1];
    }
    // Optional: playback plan file (plain text)
    std::string playback_plan_path;
    if (argc >= 3) {
        playback_plan_path = argv[2];
    }

    YAML::Node yaml_config;
    try {
        yaml_config = YAML::LoadFile(config_filepath);
    } catch (const YAML::BadFile &e) {
        std::cout << e.what() << std::endl;
        exit(-1);
    } catch (const YAML::ParserException &e) {
        std::cout << e.what() << std::endl;
        exit(-1);
    }

    Config config(yaml_config);
    std::string frame_dump_dir;
    if (const char *frame_dump_env = std::getenv("HELIXTRACK_FRAME_DIR")) {
        frame_dump_dir = frame_dump_env;
        if (!frame_dump_dir.empty()) {
            std::error_code frame_dir_ec;
            std::filesystem::create_directories(frame_dump_dir, frame_dir_ec);
            if (frame_dir_ec) {
                std::cerr << "Frame dump directory could not be created: " << frame_dump_dir << '\n';
                frame_dump_dir.clear();
            }
        }
    }

    // Load playback plan if a path was provided
    std::vector<PlaybackPlanStep> playback_plan;
    bool have_playback_plan = load_playback_plan_file(playback_plan_path, playback_plan);
    if (have_playback_plan) {
        std::cout << "Playback plan: loaded " << playback_plan.size()
                  << " step(s) from '" << playback_plan_path << "'.\n";
        for (size_t i = 0; i < playback_plan.size(); ++i) {
            std::cout << "  - rel_us=" << playback_plan[i].rel_us
                      << ", fps=" << playback_plan[i].fps
                      << ", agg_us=" << playback_plan[i].agg_us << "\n";
        }
    }

    YAML::Node ymet = yaml_config["metrics"];
    const long long  MET_SAMPLE_DT_US       = ymet["sample_dt_us"]           ? ymet["sample_dt_us"].as<long long>() : 5000;
    const double     NOISE_WIN_S            = ymet["noise_window_s"]         ? ymet["noise_window_s"].as<double>()  : 0.5;
    const double     STEADY_SLOPE_RPM_S     = ymet["steady_gt_slope_rpm_s"]  ? ymet["steady_gt_slope_rpm_s"].as<double>() : 5.0;
    const double     STEP_THRESH_RPM        = ymet["step_threshold_rpm"]     ? ymet["step_threshold_rpm"].as<double>() : 50.0;
    const double     SETTLING_TOL_PCT       = ymet["settling_tol_pct"]       ? ymet["settling_tol_pct"].as<double>() : 2.0;
    const double     RAMP_MIN_RPM_PER_S     = ymet["ramp_min_rpm_per_s"]     ? ymet["ramp_min_rpm_per_s"].as<double>() : 50.0;
    const int        EVENT_TIMING_STRIDE    = ymet["event_timing_stride"]    ? std::max(1, ymet["event_timing_stride"].as<int>()) : 1000;

    constexpr int H_RING_SAMPLES = 64;

    // --- Metrics buffers (per run / per channel) ---
    std::vector<long long> sample_ts_us, sample_dt_us_vec, latency_us_vec;
    std::vector<double>    rpm_est_vec, rpm_gt_vec, err_rpm_vec;
    std::vector<double>    samp_u_cx, samp_u_cy;
    std::vector<double>    H11_vec, H12_vec, H13_vec,
                           H21_vec, H22_vec, H23_vec,
                           H31_vec, H32_vec, H33_vec;

    // compute footprint
    std::vector<long long> per_event_us_samples;  // sampled every EVENT_TIMING_STRIDE
    std::vector<long long> solve_us, solve_batch_sizes;

    // internal: sampling and timing bases
    long long next_sample_ts_us = -1;
    long long first_stream_ts_us = -1;     // first event ts
    Clock::time_point wall_t0;
    bool wall_t0_set = false;

    long long total_expected_samples = 0;  // for dropout summary
    GroundTruthRpm ground_truth_rpm;

    // helpers for stats while streaming
    auto push_sample = [&](long long ts_us, double est_rpm, double gt_rpm) {
        if (!std::isfinite(est_rpm)) return;
        sample_ts_us.push_back(ts_us);
        rpm_est_vec.push_back(est_rpm);
        rpm_gt_vec.push_back(gt_rpm);
        if (std::isfinite(gt_rpm)) err_rpm_vec.push_back(est_rpm - gt_rpm);
        // inter-sample dt
        if (sample_ts_us.size() >= 2) sample_dt_us_vec.push_back(sample_ts_us.back() - sample_ts_us[sample_ts_us.size()-2]);
    };

    // Always load rpm_alignment if present (will NOT override start/end if already set).
    int channel_num = config.channel_num;
    load_start_and_end_us_from_rpm_alignment_if_available(config, channel_num, ground_truth_rpm);

    {
        // handle tracker - check for tracker_initial_positions directory inside the recording_filepath directory
        load_tracker_init_if_available(config, config.channel_num);
    }

    std::cout << "Opening file/camera..." << std::endl;

    Metavision::Camera camera;
    try {
        if (!config.recording_filepath.empty()) {
            Metavision::FileConfigHints file_config_hints;
            file_config_hints.real_time_playback(false);
            file_config_hints.time_shift(false);
            camera = Metavision::Camera::from_file(config.recording_filepath, file_config_hints);
        } else {
            camera = Metavision::Camera::from_first_available();
        }
    } catch (Metavision::CameraException &e) {
        // MV_LOG_ERROR() << e.what();
        return 2;
    }

    std::cout << "Camera opened. " << std::endl;

    const int camera_width  = camera.geometry().get_width();
    const int camera_height = camera.geometry().get_height();
    const int npixels       = camera_width * camera_height;

    // now we can create our frame generator using previous variables
    auto frame_gen = Metavision::PeriodicFrameGenerationAlgorithm(camera_width, camera_height, config.acc, config.fps,
        Metavision::ColorPalette::Dark);

    // Instantiate tracker (kept as-is)
    helixtrack::HelixTrackTracker tracker(camera_width, camera_height, /*B=*/config.tracker_num_blades);

    // --- Initial ablations ---------------------------------------------------
    // All ablations are percentages of the ORIGINAL values.
    //  - RPM:      rpm' = rpm * (1 + pct/100)
    //  - X shift:  x'  = x + (pct/100) * original_radius
    //  - Scale:    r'  = r * (1 + pct/100)
    //
    // NOTE: per spec, the translation uses the *original* radius, not the scaled one.
    const double ablate_rpm_pct   = config.tracker_initial_rpm_ablation;       // e.g. -10 => 0.9×
    const double ablate_x_pct     = config.tracker_initial_x_position_ablation;// e.g. -100 => x - radius
    const double ablate_x_and_radius_pct     = config.tracker_initial_x_and_radius_position_ablation;// e.g. -100 => x - radius
    const double ablate_scale_pct = config.tracker_initial_scale_ablation;     // e.g. +5 => 1.05×

    // Snapshot originals (may have been loaded from CSV already).
    const int    orig_xc     = config.tracker_init_x_center;
    const int    orig_yc     = config.tracker_init_y_center;
    const double orig_radius = config.tracker_init_radius_px;
    const double orig_psi    = config.tracker_init_psi;
    const double orig_ar     = config.tracker_init_aspect_ratio;

    // Apply pose ablations.
    // If 'x_and_radius' is set, it takes precedence and links X shift with radius scaling:
    //   x' = x + (pct/100) * orig_radius
    //   r' = r * (1 - pct/100)
    // (i.e., +10% shifts by +10% of the original radius and shrinks the radius by 10%.)
    double dx_px = 0.0;
    int    init_x_center = orig_xc;
    double init_radius_px = orig_radius;
    if (ablate_x_and_radius_pct != 0.0) {
        dx_px = (ablate_x_and_radius_pct / 100.0) * orig_radius;
        init_x_center = static_cast<int>(std::lround(orig_xc + dx_px));
        init_radius_px = orig_radius * (1.0 - ablate_x_and_radius_pct / 100.0);
    } else {
        dx_px = (ablate_x_pct / 100.0) * orig_radius;  // translation uses original radius
        init_x_center = static_cast<int>(std::lround(orig_xc + dx_px));
        init_radius_px = orig_radius * (1.0 + ablate_scale_pct / 100.0);
    }

    // 1) Initial pose (ABLATED)
    tracker.set_initial_pose(
        init_x_center,
        orig_yc,
        init_radius_px,
        orig_psi,
        orig_ar
    );

    // 2) Initial phase / RPM (RPM from YAML if >0, else GT at start), then ablate.
    const double rpm0 = (config.tracker_init_rpm > 0.0)
                        ? config.tracker_init_rpm
                        : ground_truth_rpm.at_us(config.start_us);
    const double rpm_init = std::isfinite(rpm0) ? (rpm0 * (1.0 + ablate_rpm_pct / 100.0)) : rpm0;
    tracker.set_initial_phase(rpm_init, config.tracker_init_phi0);

    // Report the actual initial values used after ablation.
    std::cout << "Init (after ablation):\n"
              << std::left << std::setw(20) << "\tpositions (px):" << init_x_center << ", " << orig_yc << '\n'
              << std::left << std::setw(20) << "\tradius (px):"    << init_radius_px << "  (orig " << orig_radius << ")\n"
              << std::left << std::setw(20) << "\tpsi:"            << orig_psi << '\n'
              << std::left << std::setw(20) << "\taspect ratio:"   << orig_ar << '\n'
              << std::left << std::setw(20) << "\tRPM estimate:"   << rpm_init << " RPM (orig " << rpm0 << ", at " << config.start_us << ')' << '\n'
              << std::left << std::setw(20) << "\tphi0:"           << config.tracker_init_phi0 << std::endl;

    if (ablate_rpm_pct != 0.0) {
        std::cout << "Applied RPM ablation: " << ablate_rpm_pct
                  << "%  => initial RPM " << rpm_init << " (from " << rpm0 << ")." << std::endl;
    }
    if (ablate_x_and_radius_pct != 0.0) {
        std::cout << "Applied combined X+Radius ablation: " << ablate_x_and_radius_pct
                  << "% of original radius (" << orig_radius << " px) => shift " << dx_px
                  << " px; x " << orig_xc << " -> " << init_x_center
                  << "; radius " << orig_radius << " -> " << init_radius_px << " px." << std::endl;
    } else {
        if (ablate_x_pct != 0.0) {
            std::cout << "Applied X-translation ablation: " << ablate_x_pct
                      << "% of original radius (" << orig_radius << " px) => shift " << dx_px
                      << " px; x " << orig_xc << " -> " << init_x_center << "." << std::endl;
        }
        if (ablate_scale_pct != 0.0) {
            std::cout << "Applied scale ablation: " << ablate_scale_pct
                      << "%  => radius " << orig_radius << " -> " << init_radius_px << " px." << std::endl;
        }
    }

    // Wire the high-level HelixTrack method components from the runtime config.
    // The tracker exposes the same names used in the paper: phase tracking,
    // radial ring gating, polarity alignment, soft annulus barriers, and
    // Balanced Tip Occupancy.

    // 3) EKF / filter controls
    tracker.set_kappa(config.tracker_kappa);
    tracker.set_q_jerk(config.tracker_q_jerk);
    tracker.set_rho(config.tracker_rho);

    // 4) Radial ROI + ring
    if (config.tracker_enable_radial_roi) tracker.enable_radial_roi(true);
    tracker.set_radial_roi(config.tracker_r_inner, config.tracker_r_outer);

    // 5) LS weights
    tracker.set_phase_term_lambda(config.tracker_lambda_phase);
    tracker.set_radial_term_lambda(config.tracker_lambda_radial);

    // 6) Visualization & auto-direction (kept switchable via YAML)
    if (config.tracker_enable_weights_visualization && config.mode == CONFIG_MODE_VISUAL) tracker.enable_weights_visualization(true);

    if (config.tracker_rotation_sign == 0)
    {
        tracker.enable_auto_direction();
        std::cout << "Tracker set to auto-rotation direction mode." << std::endl;
    }
    else
    {
        tracker.set_rotation_sign(config.tracker_rotation_sign);
        std::cout << "Tracker rotation sign set to " << config.tracker_rotation_sign << std::endl;
    }

    // 7) Polarity-phase term
    tracker.enable_polarity_phase_term(config.tracker_enable_polarity_phase_term);
    tracker.set_polarity_phase_params(
        config.tracker_lambda_pol,
        config.tracker_dtheta_pos,
        config.tracker_dtheta_neg,
        config.tracker_kappa_scale
    );

    // 8) Band-edge barrier around [r_inner, r_outer]
    tracker.set_radial_centering(config.tracker_radial_centering);
    tracker.set_band_edge_barrier(config.tracker_band_tau_u, config.tracker_band_lambda);

    // 9) Balanced Tip Occupancy (BTO)
    if (config.tracker_enable_tip_balance) tracker.enable_tip_balance(true);
    tracker.set_tip_balance_params(
        config.tracker_lambda_tip,
        config.tracker_sigma_core,
        config.tracker_delta,
        config.tracker_sigma_in,
        config.tracker_sigma_out,
        config.tracker_tau_occ,
        config.tracker_p_in_min,
        config.tracker_p_out_max
    );

    tracker.set_gn_gammas(config.gn_gamma_scale, config.gn_gamma_rotation,
        config.gn_gamma_perspective, config.gn_gamma_translation);

    std::cout << "Tracker initialized. " << std::endl;
    std::cout << "Starting with stride " << config.stride_events << " events, min_batch_size " << config.min_batch_size << "." << std::endl;

    long long first_timestep = -1;
    long long last_timestep = -1;
    long long total_events = 0;
    auto last_pose_solve = static_cast<Metavision::timestamp>(0);
    static double phi_base = 0.0;   // φ at last solve
    static double psi_base = 0.0;   // ψ at last solve
    static bool have_bases = false;
    int num_events_in_batch = 0;
    const int    B       = tracker.B();     // blade count
    long long stride_count = 0;
    std::atomic<bool> should_stop{false};

    // --- Playback plan runtime state ---
    bool plan_abs_ready = false;               // absolute timestamps computed
    size_t plan_next_idx = 0;                  // next step to apply
    auto compute_plan_abs_ts_if_needed = [&]() {
        if (!have_playback_plan || plan_abs_ready == true) return;
        if ( (config.start_us > 0) || (first_stream_ts_us >= 0) ) {
            const long long base = (config.start_us > 0) ? config.start_us : first_stream_ts_us;
            for (auto &s : playback_plan) s.abs_us = base + s.rel_us;
            std::sort(playback_plan.begin(), playback_plan.end(),
                      [](const PlaybackPlanStep& a, const PlaybackPlanStep& b){ return a.abs_us < b.abs_us; });
            plan_abs_ready = true;
            std::cout << "Playback plan: absolute base=" << base << " us computed.\n";
        }
    };

    camera.cd().add_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        if (begin == end) return;
        if (first_timestep == -1) {
            first_timestep = begin->t;
        }
        if (!wall_t0_set) { wall_t0 = Clock::now(); wall_t0_set = true; }

        const Metavision::EventCD *callback_end = end;
        if (config.end_us > 0) {
            if (begin->t >= config.end_us) {
                should_stop.store(true, std::memory_order_relaxed);
                return;
            }
            if ((end - 1)->t >= config.end_us) {
                callback_end = begin;
                while (callback_end < end && callback_end->t < config.end_us) {
                    ++callback_end;
                }
                should_stop.store(true, std::memory_order_relaxed);
                if (callback_end == begin) return;
            }
        }
        last_timestep = (callback_end - 1)->t;

        if (config.stride_events > 1) {
            stride_count++;
            if ((stride_count % config.stride_events) != 0) {
                // Even if we skip processing for stride, we may still need the base ts for plan:
                compute_plan_abs_ts_if_needed();
                return;
            }
        }

        int local_count = 0;
        for (auto it = begin; it != callback_end; ++it) {
            auto t_ev0 = Clock::now();
            tracker.process_event(it->x, it->y, it->t, int(it->p ? +1 : -1));
            if (first_stream_ts_us < 0) first_stream_ts_us = it->t; // used for plan base
            compute_plan_abs_ts_if_needed();
            ++total_events;
            ++local_count;
            // Per-event compute time sampler
            if ((total_events % EVENT_TIMING_STRIDE) == 0)
            {
                auto t_ev1 = Clock::now();
                long long dur_us = std::chrono::duration_cast<std::chrono::microseconds>(t_ev1 - t_ev0).count();
                per_event_us_samples.push_back(dur_us);
            }
            if (first_stream_ts_us < 0) first_stream_ts_us = it->t;
        }

        // Batched pose refinement is self-timed from accumulated helical phase
        // advance, while the EKF keeps phase and angular speed current at the
        // event rate.
        const double t_now   = tracker.last_time_s();
        const double phi_now = tracker.state().phase_at(t_now);
        const double psi_now = tracker.pose().q(1);
        if (!have_bases) { phi_base = phi_now; psi_base = psi_now; have_bases = true; }
        auto wrap_two_pi = [](double a){ return std::remainder(a, 2.0*M_PI); };
        const double dpsi      = wrap_two_pi(psi_now - psi_base);
        const double dphi_eff  = (phi_now - phi_base) - double(B) * dpsi;
        const double dtheta_mech = std::abs(dphi_eff / double(B));

        // Run the homography solve once the batch has enough phase support.
        if (last_timestep - last_pose_solve >= 100000 || dtheta_mech >= 1.0*M_PI) {
            num_events_in_batch = tracker.num_events_in_H_batch();
            auto t_s0 = Clock::now();
            tracker.solve_pose_and_reset(config.tracker_homography_solve_lambda_diag, /*min_batch=*/config.min_batch_size);
            auto t_s1 = Clock::now();
            long long dur_us = std::chrono::duration_cast<std::chrono::microseconds>(t_s1 - t_s0).count();
            solve_us.push_back(dur_us);
            solve_batch_sizes.push_back(num_events_in_batch);
            last_pose_solve = last_timestep;

            const double psi_after = tracker.pose().q(1);
            phi_base = phi_now;
            psi_base = psi_after;

            // std::cout << "Aspect ratio: " << tracker.estimated_aspect_ratio() <<
            //     ", psi: " << psi_after << std::endl;
        }

        // --- Periodic estimator sampling for metrics ---
        if (next_sample_ts_us < 0) {
            // start sampling at max(start_us, first_stream_ts_us)
            long long s0 = (config.start_us > 0) ? config.start_us : first_stream_ts_us;
            next_sample_ts_us = s0;
        }
        while (last_timestep >= next_sample_ts_us) {
            // Estimator snapshot (use same RPM as your HUD)
            auto st = tracker.state();
            double est_rpm = std::abs(st.omega) * 60.0 / (2.0 * M_PI);
            double gt_rpm  = ground_truth_rpm.at_us(next_sample_ts_us);

            // Latency vs stream time (lower bound on e2e)
            long long stream_us = next_sample_ts_us - first_stream_ts_us;
            long long now_us = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - wall_t0).count();
            long long latency_us = now_us - stream_us;
            latency_us_vec.push_back(latency_us);

            push_sample(next_sample_ts_us, est_rpm, gt_rpm);

            // ---- (B) Homography & u-center sampled at the same instant ----
            // Collect ring sample points (in image pixels) without drawing to a frame.
            std::vector<cv::Point2f> ring_pts;
            ring_pts.reserve(H_RING_SAMPLES);
            auto collect = [&](int x, int y, float w, const cv::Vec3f&) {
                // Only keep reasonably confident samples; w is in [0,1] typically.
                if (w > 0.f)
                    ring_pts.emplace_back((float)x, (float)y);
            };
            tracker.draw_predicted_ring(collect, /*samples=*/H_RING_SAMPLES);
            if ((int)ring_pts.size() > H_RING_SAMPLES) {
                ring_pts.resize(H_RING_SAMPLES);
            }

            // u center as mean of ring points
            double cx = std::numeric_limits<double>::quiet_NaN();
            double cy = std::numeric_limits<double>::quiet_NaN();
            if (!ring_pts.empty()) {
                long double sx=0, sy=0;
                for (auto &p : ring_pts) { sx += p.x; sy += p.y; }
                cx = double(sx / (long double)ring_pts.size());
                cy = double(sy / (long double)ring_pts.size());
            }
            samp_u_cx.push_back(cx);
            samp_u_cy.push_back(cy);

            // Estimate H that maps unit circle -> predicted ring (pixel).
            auto H = tracker.pose().H;
            H11_vec.push_back(H(0,0)); H12_vec.push_back(H(0,1)); H13_vec.push_back(H(0,2));
            H21_vec.push_back(H(1,0)); H22_vec.push_back(H(1,1)); H23_vec.push_back(H(1,2));
            H31_vec.push_back(H(2,0)); H32_vec.push_back(H(2,1)); H33_vec.push_back(H(2,2));


            next_sample_ts_us += MET_SAMPLE_DT_US;
            ++total_expected_samples;

            // print elapse seconds every second
            if (((last_timestep - first_timestep) % 1000000) == 0)
            {
                std::cout << "Elapsed: " << last_timestep - first_timestep << " us" << std::endl;
            }
        }

        // ------------- Frame generation with playback-plan switching -------------
        if (config.mode == CONFIG_MODE_VISUAL) {
            // If no plan or not yet initialized, just pass through the whole buffer.
            if (!have_playback_plan || !plan_abs_ready || plan_next_idx >= playback_plan.size() ||
                (callback_end - 1)->t < playback_plan[plan_next_idx].abs_us) {
                frame_gen.process_events(begin, callback_end);
            } else {
                // We have at least one pending step inside [begin, end).
                const Metavision::EventCD* cursor = begin;
                while (cursor < callback_end && plan_next_idx < playback_plan.size()) {
                    const long long target = playback_plan[plan_next_idx].abs_us;
                    // If next step is beyond this buffer, flush remaining events and stop.
                    if ((callback_end - 1)->t < target) {
                        break;
                    }
                    // Find the first event with t >= target (cut)
                    const Metavision::EventCD* cut = cursor;
                    while (cut < callback_end && cut->t < target) ++cut;

                    // Choose the nearest event in [cursor, end) to target.
                    // Default: cut (first >= target). If there is a previous event and it's closer, use it.
                    const Metavision::EventCD* apply_it = cut;
                    if (cut > cursor) {
                        const Metavision::EventCD* prev = cut - 1;
                        // diffs are non-negative by construction
                        const long long diff_prev = target - prev->t;
                        const long long diff_cut  = (cut < callback_end) ? (cut->t - target) : std::numeric_limits<long long>::max();
                        if (cut == callback_end || diff_prev <= diff_cut) {
                            apply_it = prev;
                        }
                    }

                    // Process events up to (but not including) 'apply_it'
                    if (apply_it > cursor) {
                        frame_gen.process_events(cursor, apply_it);
                    }

                    // Apply the plan change *right before* the chosen event is consumed
                    if (plan_next_idx < playback_plan.size()) {
                        const auto &s = playback_plan[plan_next_idx];
                        if (s.fps > 0.0) {
                            frame_gen.set_fps(s.fps);
                        }
                        // Metavision API: change accumulation/aggregation time (µs).
                        frame_gen.set_accumulation_time_us(s.agg_us);
                        std::cout << "Playback plan: switch @ts=" << (apply_it < callback_end ? apply_it->t : (callback_end - 1)->t)
                                  << " (target=" << s.abs_us << ", rel=" << s.rel_us
                                  << ")  fps=" << s.fps
                                  << "  aggregation_us=" << s.agg_us << "\n";
                        // mark applied and advance
                        // (we do not set s.applied=true by const; plan_next_idx guards progress)
                        ++plan_next_idx;
                    }
                    // Continue from the apply point (which has not yet been fed to frame_gen)
                    cursor = apply_it;
                }
                // Flush any remaining events in this buffer
                if (cursor < callback_end) {
                    frame_gen.process_events(cursor, callback_end);
                }
            }
        }
    });

    // config.end_us = config.start_us + 1000000; // TODO: run for 1 second only for testing

    Metavision::Window * window = nullptr;
    if (config.mode == CONFIG_MODE_VISUAL)
    {
        window = new Metavision::Window("Recording", camera_width, camera_height, Metavision::BaseWindow::RenderMode::BGR);

        // weights overlay UI state (visible to both keyboard + frame callbacks)
        static bool  weights_on    = false;
        static bool  polarity_on    = false;
        static int   weights_mode  = 0;                         // 0=Total, 1=Phase, 2=Radial
        static int   weights_cmap  = cv::COLORMAP_VIRIDIS;      // cycle with 'C'
        static float weights_alpha = 0.65f;                     // '[' / ']'

        window->set_keyboard_callback(
        [&, window](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods)
        {
            if (action == Metavision::UIAction::RELEASE &&
                (key == Metavision::UIKeyEvent::KEY_ESCAPE || key == Metavision::UIKeyEvent::KEY_Q))
            {
                window->set_close_flag();
            }
            if (action == Metavision::UIAction::RELEASE) {
                switch (key) {
                    case Metavision::UIKeyEvent::KEY_W: weights_on = !weights_on; break;
                    case Metavision::UIKeyEvent::KEY_P: polarity_on = !polarity_on; break;
                    case Metavision::UIKeyEvent::KEY_1: weights_mode = 0; break;
                    case Metavision::UIKeyEvent::KEY_2: weights_mode = 1; break;
                    case Metavision::UIKeyEvent::KEY_3: weights_mode = 2; break;
                    case Metavision::UIKeyEvent::KEY_4: weights_mode = 3; break; // Polarity
                    case Metavision::UIKeyEvent::KEY_5: weights_mode = 4; break; // BandIn
                    case Metavision::UIKeyEvent::KEY_6: weights_mode = 5; break; // BandOut
                    case Metavision::UIKeyEvent::KEY_7: weights_mode = 6; break; // EKF
                    case Metavision::UIKeyEvent::KEY_8: weights_mode = 7; break; // whalf

                    case Metavision::UIKeyEvent::KEY_LEFT_BRACKET:
                        weights_alpha = std::max(0.05f, weights_alpha - 0.05f); break;
                    case Metavision::UIKeyEvent::KEY_RIGHT_BRACKET:
                        weights_alpha = std::min(0.95f, weights_alpha + 0.05f); break;
                    case Metavision::UIKeyEvent::KEY_C: {
                        static const int cmaps[] = {
                            cv::COLORMAP_VIRIDIS, cv::COLORMAP_MAGMA, cv::COLORMAP_INFERNO,
                            cv::COLORMAP_PLASMA,  cv::COLORMAP_JET
                        };
                        static int idx = 0;
                        idx = (idx + 1) % (int)(sizeof(cmaps)/sizeof(cmaps[0]));
                        weights_cmap = cmaps[idx];
                        break;
                    }
                    case Metavision::UIKeyEvent::KEY_PAGE_UP:
                        {
                            frame_gen.set_fps(frame_gen.get_fps() * 0.5);
                            break;
                        }
                    case Metavision::UIKeyEvent::KEY_PAGE_DOWN:
                        {
                            frame_gen.set_fps(frame_gen.get_fps() * 2.0);
                            break;
                        }
                    case Metavision::UIKeyEvent::KEY_MINUS:
                        break;
                    case Metavision::UIKeyEvent::KEY_EQUAL: // usually '+' with Shift
                        break;
                    default: break;
                }
            }
        });

        long long image_count = 0;

        frame_gen.set_output_callback([&, window](Metavision::timestamp mtv_ts, cv::Mat& frame){
            if (weights_on) {
                tracker.draw_event_weights_cv(
                    frame,
                    weights_alpha,
                    weights_mode,
                    weights_cmap,
                    /*log_scale=*/true,
                    /*clip_sigma=*/3.0f
                );
            }

            // Render predicted ring overlay
            auto drawpx = [&](int x, int y, float w, const cv::Vec3f& rgb01) {
                if (x < 0 || x >= camera_width || y < 0 || y >= camera_height) return;
                // Clamp w to [0,1] and scale to 0..255
                const float a   = std::max(0.f, std::min(1.f, w));
                const float amp = 255.f * a;

                // Map RGB -> BGR for OpenCV
                const float addB = amp * std::max(0.f, std::min(1.f, rgb01[2])); // from rgb01.b
                const float addG = amp * std::max(0.f, std::min(1.f, rgb01[1])); // from rgb01.g
                const float addR = amp * std::max(0.f, std::min(1.f, rgb01[0])); // from rgb01.r

                for (int xx = x-1; xx <= x+1; ++xx) {
                    for (int yy = y-1; yy <= y+1; ++yy) {
                        if (xx < 0 || xx >= camera_width || yy < 0 || yy >= camera_height) continue;
                        cv::Vec3b& pix = frame.at<cv::Vec3b>(yy, xx);
                        // Additive blend with clamp
                        int B = int(pix[0]) + int(addB + 0.5f);
                        int G = int(pix[1]) + int(addG + 0.5f);
                        int R = int(pix[2]) + int(addR + 0.5f);
                        pix[0] = (uchar)std::min(255, B);
                        pix[1] = (uchar)std::min(255, G);
                        pix[2] = (uchar)std::min(255, R);
                    }
                }
                // cv::Vec3b& pix = frame.at<cv::Vec3b>(y, x);
                // // Additive blend with clamp
                // int B = int(pix[0]) + int(addB + 0.5f);
                // int G = int(pix[1]) + int(addG + 0.5f);
                // int R = int(pix[2]) + int(addR + 0.5f);

                // pix[0] = (uchar)std::min(255, B);
                // pix[1] = (uchar)std::min(255, G);
                // pix[2] = (uchar)std::min(255, R);
            };
            tracker.draw_predicted_ring(drawpx, /*samples=*/1080);

            if (polarity_on)
            {
                // polarity onset/offset guide lines (orange=onset +, cyan=offset −)
                tracker.draw_polarity_guides(
                    drawpx,
                    /*radial_samples=*/540,
                    /*col_pos=*/cv::Vec3f(1.f, 0.55f, 0.f),
                    /*col_neg=*/cv::Vec3f(0.f, 0.65f, 1.f),
                    /*thickness_px=*/1
                );
            }

            // HUD text
            auto st = tracker.state();
            double t = tracker.last_time_s();
            double rpm = std::abs(st.omega) * 60.0 / (2.0 * 3.141592653589793);
            // Ground-truth RPM @ current frame timestamp
            double gt_rpm = ground_truth_rpm.at_us(mtv_ts);
            double diff_relative = std::numeric_limits<double>::quiet_NaN();
            const char* mode_str = "Tot";
            if      (weights_mode==1) mode_str = "Phi";
            else if (weights_mode==2) mode_str = "Radial";
            else if (weights_mode==3) mode_str = "Pol";
            else if (weights_mode==4) mode_str = "BandIn";
            else if (weights_mode==5) mode_str = "BandOut";
            else if (weights_mode==6) mode_str = "EKF";
            if (std::isfinite(gt_rpm)) diff_relative = (rpm - gt_rpm) / gt_rpm * 100.0;
            std::ostringstream oss;
            oss << "d=2m, M_ego=1, t=" << std::fixed << std::setprecision(3) << t;
            oss << "s  estimated RPM=" << std::setprecision(1) << rpm;
            if (std::isfinite(diff_relative)) {
                oss << " (" << (diff_relative >= 0.0 ? "+" : "") << std::setprecision(1) << diff_relative << "% difference from GT RPM)";
            }
            // oss << " dPhi=" << std::setprecision(2) << dtheta_mech
            // oss << " " << mode_str
                // << "E=" << num_events_in_batch;
            cv::putText(frame, oss.str(), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 1, cv::LINE_AA);
            if (!frame_dump_dir.empty()) {
                const auto output_image_ref_name =
                    std::filesystem::path(frame_dump_dir) /
                    (std::to_string(image_count) + "_ref.png");
                cv::imwrite(output_image_ref_name.string(), frame);
            }
            ++image_count;

                window->show(frame);
        });

        std::cout << "Visual mode initialized." << std::endl;
    }

    if (config.start_us > 0)
    {
        std::cout << "Seeking to " << config.start_us << " us..." << std::endl;
        while (!camera.offline_streaming_control().seek(config.start_us))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "Seeked to " << config.start_us << " us." << std::endl;
    }
    std::cout << "Starting camera..." << std::endl;
    camera.start();
    std::cout << "Camera started." << std::endl;

    // keep running until the camera is off, the recording is finished or the escape key was pressed
    while (camera.is_running() &&
           (window == nullptr || !window->should_close()) &&
           !should_stop.load(std::memory_order_relaxed)) {
        if (config.mode == CONFIG_MODE_VISUAL)
        {
            // we poll events (keyboard, mouse etc.) from the system with a 20ms sleep to avoid using 100% of a CPU's core
            // and we push them into the window where the callback on the escape key will ask the windows to close
            static constexpr std::int64_t kSleepPeriodMs = 20;
            Metavision::EventLoop::poll_and_dispatch(kSleepPeriodMs);
        } else {
            // In textual mode there is no UI event loop to block on. Yield a
            // little so the offline decoding and callback threads can make
            // forward progress instead of competing with a tight busy-spin.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    camera.stop();

    delete window;


    write_run_metrics(
        config,
        RunMetricsConfig{
            MET_SAMPLE_DT_US,
            NOISE_WIN_S,
            STEADY_SLOPE_RPM_S,
            STEP_THRESH_RPM,
            SETTLING_TOL_PCT,
            RAMP_MIN_RPM_PER_S,
        },
        RunMetricsData{
            sample_ts_us,
            sample_dt_us_vec,
            latency_us_vec,
            rpm_est_vec,
            rpm_gt_vec,
            err_rpm_vec,
            samp_u_cx,
            samp_u_cy,
            H11_vec,
            H12_vec,
            H13_vec,
            H21_vec,
            H22_vec,
            H23_vec,
            H31_vec,
            H32_vec,
            H33_vec,
            per_event_us_samples,
            solve_us,
            solve_batch_sizes,
            total_expected_samples,
        });

    return 0;
}

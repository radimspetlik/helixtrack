#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <metavision/sdk/base/utils/timestamp.h>
#include <yaml-cpp/yaml.h>

struct Config {
    explicit Config(const YAML::Node &y)
        : width(y["width"].as<unsigned short>()),
          height(y["height"].as<unsigned short>()),
          tracker_init_x_center(y["tracker_init_x_center"].as<int>()),
          tracker_init_y_center(y["tracker_init_y_center"].as<int>()),
          fps(y["fps"].as<double>()),
          mode(y["mode"].as<std::string>()),
          analysis_filepath(y["analysis_filepath"].as<std::string>()),
          tensorboard_log_file(y["tensorboard_log_file"].as<std::string>()),
          recording_filepath(y["recording_filepath"].as<std::string>()),
          acc(y["acc"].as<std::uint32_t>()),
          start_us(y["start_us"].as<Metavision::timestamp>()),
          end_us(y["end_us"].as<Metavision::timestamp>()),
          channel_num(y["channel_num"].as<int>()) {
        if (y["tracker_init_radius_px"]) tracker_init_radius_px = y["tracker_init_radius_px"].as<float>();
        if (y["tracker_init_psi"]) tracker_init_psi = y["tracker_init_psi"].as<float>();
        if (y["tracker_init_rpm"]) tracker_init_rpm = y["tracker_init_rpm"].as<double>();
        if (y["tracker_init_phi0"]) tracker_init_phi0 = y["tracker_init_phi0"].as<double>();

        if (y["tracker_kappa"]) tracker_kappa = y["tracker_kappa"].as<float>();
        if (y["tracker_q_jerk"]) tracker_q_jerk = y["tracker_q_jerk"].as<float>();
        if (y["tracker_rho"]) tracker_rho = y["tracker_rho"].as<float>();

        if (y["tracker_enable_radial_roi"]) tracker_enable_radial_roi = y["tracker_enable_radial_roi"].as<bool>();
        if (y["tracker_r_inner"]) tracker_r_inner = y["tracker_r_inner"].as<float>();
        if (y["tracker_r_outer"]) tracker_r_outer = y["tracker_r_outer"].as<float>();

        if (y["tracker_lambda_phase"]) tracker_lambda_phase = y["tracker_lambda_phase"].as<float>();
        if (y["tracker_lambda_radial"]) tracker_lambda_radial = y["tracker_lambda_radial"].as<float>();

        if (y["tracker_enable_weights_visualization"]) {
            tracker_enable_weights_visualization = y["tracker_enable_weights_visualization"].as<bool>();
        }

        if (y["tracker_rotation_sign"]) tracker_rotation_sign = y["tracker_rotation_sign"].as<int>();

        if (y["tracker_enable_polarity_phase_term"]) {
            tracker_enable_polarity_phase_term = y["tracker_enable_polarity_phase_term"].as<bool>();
        }
        if (y["tracker_lambda_pol"]) tracker_lambda_pol = y["tracker_lambda_pol"].as<float>();
        if (y["tracker_dtheta_pos"]) tracker_dtheta_pos = y["tracker_dtheta_pos"].as<float>();
        if (y["tracker_dtheta_neg"]) tracker_dtheta_neg = y["tracker_dtheta_neg"].as<float>();
        if (y["tracker_kappa_scale"]) tracker_kappa_scale = y["tracker_kappa_scale"].as<float>();

        if (y["tracker_radial_centering"]) tracker_radial_centering = y["tracker_radial_centering"].as<bool>();
        if (y["tracker_band_tau_u"]) tracker_band_tau_u = y["tracker_band_tau_u"].as<float>();
        if (y["tracker_band_lambda"]) tracker_band_lambda = y["tracker_band_lambda"].as<float>();

        if (y["tracker_enable_tip_balance"]) tracker_enable_tip_balance = y["tracker_enable_tip_balance"].as<bool>();
        if (y["tracker_lambda_tip"]) tracker_lambda_tip = y["tracker_lambda_tip"].as<float>();
        if (y["tracker_sigma_core"]) tracker_sigma_core = y["tracker_sigma_core"].as<float>();
        if (y["tracker_delta"]) tracker_delta = y["tracker_delta"].as<float>();
        if (y["tracker_sigma_in"]) tracker_sigma_in = y["tracker_sigma_in"].as<float>();
        if (y["tracker_sigma_out"]) tracker_sigma_out = y["tracker_sigma_out"].as<float>();
        if (y["tracker_tau_occ"]) tracker_tau_occ = y["tracker_tau_occ"].as<float>();
        if (y["tracker_p_in_min"]) tracker_p_in_min = y["tracker_p_in_min"].as<float>();
        if (y["tracker_p_out_max"]) tracker_p_out_max = y["tracker_p_out_max"].as<float>();
        if (y["tracker_homography_solve_lambda_diag"]) {
            tracker_homography_solve_lambda_diag = y["tracker_homography_solve_lambda_diag"].as<float>();
        }
        if (y["tracker_init_aspect_ratio"]) tracker_init_aspect_ratio = y["tracker_init_aspect_ratio"].as<float>();
        if (y["tracker_num_blades"]) tracker_num_blades = y["tracker_num_blades"].as<int>();
        if (y["tracker_gn_gamma_translation"]) gn_gamma_translation = y["tracker_gn_gamma_translation"].as<float>();
        if (y["tracker_gn_gamma_scale"]) gn_gamma_scale = y["tracker_gn_gamma_scale"].as<float>();
        if (y["tracker_gn_gamma_rotation"]) gn_gamma_rotation = y["tracker_gn_gamma_rotation"].as<float>();
        if (y["tracker_gn_gamma_perspective"]) gn_gamma_perspective = y["tracker_gn_gamma_perspective"].as<float>();
        if (y["stride_events"]) stride_events = y["stride_events"].as<int>();
        if (y["min_batch_size"]) min_batch_size = y["min_batch_size"].as<std::size_t>();
        if (y["tracker_initial_rpm_ablation"]) {
            tracker_initial_rpm_ablation = y["tracker_initial_rpm_ablation"].as<float>();
        }
        if (y["tracker_initial_x_position_ablation"]) {
            tracker_initial_x_position_ablation = y["tracker_initial_x_position_ablation"].as<float>();
        }
        if (y["tracker_initial_scale_ablation"]) {
            tracker_initial_scale_ablation = y["tracker_initial_scale_ablation"].as<float>();
        }
        if (y["tracker_initial_x_and_radius_position_ablation"]) {
            tracker_initial_x_and_radius_position_ablation =
                y["tracker_initial_x_and_radius_position_ablation"].as<float>();
        }
    }

    unsigned short width{0};
    unsigned short height{0};
    double fps{0.0};
    int tracker_init_x_center{0};
    int tracker_init_y_center{0};

    std::string mode;
    std::string analysis_filepath;
    std::string tensorboard_log_file;
    std::string recording_filepath;
    std::uint32_t acc{0};
    Metavision::timestamp start_us{0};
    Metavision::timestamp end_us{0};
    int channel_num{0};

    float tracker_init_radius_px{36.f};
    float tracker_init_psi{0.0f};
    double tracker_init_rpm{0.0};
    double tracker_init_phi0{0.0};

    float tracker_kappa{22.0f};
    float tracker_q_jerk{100.0f};
    float tracker_rho{0.5f};

    bool tracker_enable_radial_roi{true};
    float tracker_r_inner{0.3f};
    float tracker_r_outer{1.3f};

    float tracker_lambda_phase{1.0f};
    float tracker_lambda_radial{1.0f};

    bool tracker_enable_weights_visualization{true};

    bool tracker_enable_polarity_phase_term{true};
    float tracker_lambda_pol{1.0f};
    float tracker_dtheta_pos{0.25f};
    float tracker_dtheta_neg{-0.25f};
    float tracker_kappa_scale{0.25f};

    bool tracker_radial_centering{true};
    float tracker_band_tau_u{0.05f};
    float tracker_band_lambda{8.0f};

    bool tracker_enable_tip_balance{true};
    float tracker_lambda_tip{3.0f};
    float tracker_sigma_core{0.06f};
    float tracker_delta{0.03f};
    float tracker_sigma_in{0.02f};
    float tracker_sigma_out{0.02f};
    float tracker_tau_occ{0.05f};
    float tracker_p_in_min{0.50f};
    float tracker_p_out_max{0.50f};
    float tracker_homography_solve_lambda_diag{1e-3f};
    int tracker_rotation_sign{0};
    float tracker_init_aspect_ratio{1.0f};
    int tracker_num_blades{2};
    float gn_gamma_translation{1.0f};
    float gn_gamma_scale{1.0f};
    float gn_gamma_rotation{1.0f};
    float gn_gamma_perspective{1.0f};
    int stride_events{1};
    std::size_t min_batch_size{0};
    float tracker_initial_rpm_ablation{0.0f};
    float tracker_initial_x_position_ablation{0.0f};
    float tracker_initial_scale_ablation{0.0f};
    float tracker_initial_x_and_radius_position_ablation{0.0f};
};

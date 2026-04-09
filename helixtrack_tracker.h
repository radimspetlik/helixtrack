#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "helixtrack_heatmaps.h"
#include "helixtrack_types.h"

namespace helixtrack {

// HelixTrackTracker is the official HelixTrack implementation core:
// a per-event phase tracker coupled with batched homography refinement.
class HelixTrackTracker {
public:
    HelixTrackTracker(int im_w, int im_h, int B = 2)
        : width_(im_w), height_(im_h) {
        Vec2f t(0.5f * im_w, 0.5f * im_h);
        const float s = 0.25f * std::min(im_w, im_h);
        pose_.setSimilarity(s, 0.f, t);

        P_.setZero();
        P_(0, 0) = 1.0;
        P_(1, 1) = 100.0;
        P_(2, 2) = 10000.0;

        set_blade_count(B);
    }

    HelixTrackTracker(int im_w, int im_h, int B, float init_psi, float init_aspect)
        : HelixTrackTracker(im_w, im_h, B) {
        Vec2f t(0.5f * im_w, 0.5f * im_h);
        const float s = 0.25f * std::min(im_w, im_h);
        pose_.setSimilarityWithAspect(s, init_psi, t, init_aspect);
    }

    void set_initial_pose(float cx, float cy, float radius_pixels, float psi_rad = 0.f) {
        pose_.setSimilarity(std::max(10.f, radius_pixels), psi_rad, Vec2f(cx, cy));
    }

    void set_initial_pose(float cx, float cy, float radius_pixels, float psi_rad, float aspect_ratio) {
        pose_.setSimilarityWithAspect(radius_pixels, psi_rad, Vec2f(cx, cy), aspect_ratio);
    }

    float estimated_aspect_ratio() const {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        return pose_.unit_circle_aspect();
    }

    void set_initial_phase(double rpm, double phi0_rad = 0.0) {
        state_.omega = rpm_to_rad_per_sec(rpm);
        state_.phi0 = phi0_rad;
        phi_now_ = phi0_rad;
    }

    void set_q_jerk(float jerk) { q_jerk_ = std::max(0.f, jerk); }
    void set_kappa(float kappa) { state_.kappa = kappa; }
    void set_blade_count(int B) { B_ = B; }
    void enable_radial_roi(bool on = true) { radial_roi_enabled_ = on; }

    void set_radial_roi(float r_inner, float r_outer) {
        r_inner_ = std::max(0.f, r_inner);
        r_outer_ = std::max(r_inner_, r_outer);
    }

    void enable_polarity_phase_term(bool on = true) { pol_phase_on_ = on; }

    void set_polarity_phase_params(float lambda_pol, float dtheta_pos, float dtheta_neg,
                                   float kappa_scale = 1.0f) {
        lambda_pol_ = std::max(0.f, lambda_pol);
        delta_pol_pos_ = dtheta_pos;
        delta_pol_neg_ = dtheta_neg;
        kappa_pol_scale_ = std::max(0.f, kappa_scale);
    }

    void set_radial_centering(bool cond) { band_barrier_on_ = cond; }

    void set_band_edge_barrier(float tau, float lambda) {
        tau_band_ = std::max(1e-6f, tau);
        lambda_band_ = std::max(0.f, lambda);
    }

    void set_phase_term_lambda(float lambda) { lambda_phase_ = std::max(0.f, lambda); }
    void set_radial_term_lambda(float lambda) { lambda_rad_ = std::max(0.f, lambda); }

    int B() const { return B_; }
    void set_rho(float rho) { rho_ = std::clamp(rho, 0.0f, 1.0f); }

    void enable_gpu_accum(bool on = true) { gpu_accum_on_ = on; }

    void apply_pose_increment_ext(const Eigen::Matrix<float, 6, 1> &dq) {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        pose_.applyIncrement(dq);
    }

    void set_rotation_sign(int s) {
        rotation_sign_.store((s >= 0 ? +1 : -1), std::memory_order_relaxed);
    }

    int rotation_sign() const { return rotation_sign_.load(std::memory_order_relaxed); }
    void enable_auto_direction(bool on = true) { auto_dir_on_ = on; }

    void enable_tip_balance(bool on = true) { tip_balance_on_ = on; }

    void set_tip_balance_params(float lambda_tip, float sigma_core, float delta, float sigma_in,
                                float sigma_out, float tau_occ, float p_in_min = 0.50f,
                                float p_out_max = 0.50f) {
        lambda_tip_ = std::max(0.f, lambda_tip);
        sigma_core_ = std::max(1e-4f, sigma_core);
        delta_tip_ = std::max(1e-4f, delta);
        sigma_in_ = std::max(1e-4f, sigma_in);
        sigma_out_ = std::max(1e-4f, sigma_out);
        tau_occ_ = std::max(1e-6f, tau_occ);
        p_in_min_ = std::clamp(p_in_min, 0.f, 1.f);
        p_out_max_ = std::clamp(p_out_max, 0.f, 1.f);
    }

    void set_gn_gammas(float gamma_scale, float gamma_rotation, float gamma_perspective,
                       float gamma_translation) {
        gn_gamma_scale_ = std::max(0.f, gamma_scale);
        gn_gamma_rotation_ = std::max(0.f, gamma_rotation);
        gn_gamma_perspective_ = std::max(0.f, gamma_perspective);
        gn_gamma_translation_ = std::max(0.f, gamma_translation);

        std::cout << "GN gammas set to: "
                  << " scale=" << gn_gamma_scale_
                  << " rotation=" << gn_gamma_rotation_
                  << " perspective=" << gn_gamma_perspective_
                  << " translation=" << gn_gamma_translation_ << std::endl;
    }

    template <typename Drawer>
    void draw_u_circle(Drawer &&draw_pixel, float r, int samples = 720,
                       cv::Vec3f color = cv::Vec3f(1.f, 1.f, 1.f)) const {
        PoseH Hsnap;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            Hsnap = pose_;
        }
        for (int k = 0; k < samples; ++k) {
            const float th = kTwoPI * (k / float(samples));
            const Eigen::Vector2f u(r * std::cos(th), r * std::sin(th));
            const Eigen::Vector2f x = Hsnap.x_from_u(u);
            draw_pixel(int(std::lround(x.x())), int(std::lround(x.y())), 1.f, color);
        }
    }

    inline float delta(const Eigen::Vector2f &u, float phi) const {
        const float r = std::max(1e-6f, u.norm());
        const float theta = std::atan2(u.y(), u.x());
        const float helix = B_ * theta + beta_.eval(r);
        return wrap_pi(phi - helix);
    }

    inline float circularGaussianBand(const Eigen::Vector2f &u, float mu = 1.0, float sigma = 0.15f) {
        const float r = std::hypot(u.x(), u.y());
        return circularGaussianBandU(r, mu, sigma);
    }

    inline float circularGaussianBandU(float u, float mu = 1.0, float sigma = 0.15f) {
        const float z = (u - mu) / sigma;
        return std::exp(-0.5f * z * z);
    }

    template <typename Drawer>
    void draw_polarity_guides(Drawer &&draw_pixel, int radial_samples = 360,
                              cv::Vec3f col_pos = cv::Vec3f(1.f, 0.55f, 0.f),
                              cv::Vec3f col_neg = cv::Vec3f(0.f, 0.65f, 1.f),
                              int thickness_px = 1) {
        if (!pol_phase_on_) return;

        PoseH Hsnap;
        State Ssnap;
        float r_in, r_out, r_max;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            Hsnap = pose_;
            Ssnap = state_;
            r_in = r_inner_;
            r_out = r_outer_;
        }
        r_in = std::max(0.0f, r_in);
        r_max = std::max(r_in, r_out);

        const int sgn = rotation_sign_;
        const int B = std::max(1, B_);
        const float dthB = helixtrack::kTwoPI / float(B);
        const double t = last_time_s_;
        const double psi = double(Hsnap.q(1));
        const double phi_eff = Ssnap.phase_at(t) - double(B) * psi;

        auto draw_thick = [&](int x, int y, const cv::Vec3f &rgb) {
            for (int dy = -thickness_px; dy <= thickness_px; ++dy) {
                for (int dx = -thickness_px; dx <= thickness_px; ++dx) {
                    if (std::abs(dx) + std::abs(dy) > thickness_px) continue;
                    draw_pixel(x + dx, y + dy, 1.f, rgb);
                }
            }
        };

        for (int i = 0; i < radial_samples; ++i) {
            const float a = (radial_samples <= 1) ? 0.f : (float(i) / float(radial_samples - 1));
            const float r = r_in + a * (r_max - r_in);
            const float bet = beta_.eval(r);

            const float th_base_pos = float((phi_eff - double(bet) - double(delta_pol_pos_)) / double(B));
            const float th_base_neg = float((phi_eff - double(bet) - double(delta_pol_neg_)) / double(B));

            for (int k = 0; k < B; ++k) {
                const float th_pos_k = helixtrack::wrap_pi(th_base_pos + k * dthB);
                const float th_neg_k = helixtrack::wrap_pi(th_base_neg + k * dthB);

                const Eigen::Vector2f u_pos(r * std::cos(th_pos_k), float(sgn) * r * std::sin(th_pos_k));
                const Eigen::Vector2f u_neg(r * std::cos(th_neg_k), float(sgn) * r * std::sin(th_neg_k));

                const Eigen::Vector2f x_pos = Hsnap.x_from_u(u_pos);
                const Eigen::Vector2f x_neg = Hsnap.x_from_u(u_neg);

                const int xp = int(std::lround(x_pos.x()));
                const int yp = int(std::lround(x_pos.y()));
                const int xn = int(std::lround(x_neg.x()));
                const int yn = int(std::lround(x_neg.y()));

                draw_thick(xp, yp, col_pos);
                draw_thick(xn, yn, col_neg);
            }
        }
    }

    void process_event(int x, int y, std::int64_t t_us, int polarity) {
        // Per-event phase tracking starts with the constant-acceleration state
        // model with white jerk process noise.
        if (!t0_set_) {
            t0_set_ = true;
            t0_us_ = t_us;
        }
        const double t = 1e-6 * double(t_us - t0_us_);
        if (last_kf_time_ < 0.0) last_kf_time_ = t;
        double dt = t - last_kf_time_;
        if (dt < 0.0) dt = 0.0;

        phi_now_ += state_.omega * dt + 0.5 * state_.alpha * dt * dt;
        state_.omega += state_.alpha * dt;

        Eigen::Matrix3d F;
        F << 1.0, dt, 0.5 * dt * dt, 0.0, 1.0, dt, 0.0, 0.0, 1.0;
        const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
        Eigen::Matrix3d Qd;
        Qd << dt5 / 20.0, dt4 / 8.0, dt3 / 6.0, dt4 / 8.0, dt3 / 3.0, dt2 / 2.0,
            dt3 / 6.0, dt2 / 2.0, dt;
        Qd *= q_jerk_;
        P_ = F * P_ * F.transpose() + Qd;
        last_kf_time_ = t;

        PoseH Hsnap;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            Hsnap = pose_;
        }
        const float psi = Hsnap.q(1);
        const int sgn = rotation_sign_.load(std::memory_order_relaxed);

        const Eigen::Vector2f xpix{float(x), float(y)};
        const Eigen::Vector2f u = Hsnap.u_from_x(xpix);

        const float unorm = u.norm();
        if (radial_roi_enabled_) {
            if (unorm < r_inner_ || unorm > r_outer_) return;
        }

        const double phi_eff = state_.phase_at(t) - double(B_) * double(psi);

        // Wrapped helical phase residual built from effective phase and the
        // signed rotor-plane azimuth of the event.
        const auto theta_dir = std::atan2(double(sgn) * double(u.y()), double(u.x()));
        const float del = helixtrack::wrap_pi(float(phi_eff - (double(B_) * theta_dir) - beta_.eval(unorm)));

        if (auto_dir_on_) {
            const float del_pos = helixtrack::wrap_pi(float((state_.phase_at(t) - double(+1 * B_) * double(psi)) -
                                                       (double(B_) * std::atan2(+1.0 * double(u.y()), double(u.x()))) -
                                                       beta_.eval(unorm)));
            const float del_neg = helixtrack::wrap_pi(float((state_.phase_at(t) - double(-1 * B_) * double(psi)) -
                                                       (double(B_) * std::atan2(-1.0 * double(u.y()), double(u.x()))) -
                                                       beta_.eval(unorm)));

            const float w_vm_pos = std::exp(state_.kappa * (std::cos(del_pos) - 1.f));
            const float w_vm_neg = std::exp(state_.kappa * (std::cos(del_neg) - 1.f));
            score_pos_ += w_vm_pos;
            score_neg_ += w_vm_neg;

            if (++auto_dir_counter_ >= auto_dir_period_) {
                if (score_pos_ > score_neg_ * (1.0 + auto_dir_margin_)) rotation_sign_ = +1;
                else if (score_neg_ > score_pos_ * (1.0 + auto_dir_margin_)) rotation_sign_ = -1;
                std::cout << "[HelixTrack] Auto direction scores: +1 -> " << score_pos_
                          << ", -1 -> " << score_neg_
                          << ", set direction " << (rotation_sign_.load() >= 0 ? "+1" : "-1") << std::endl;
                auto_dir_counter_ = 0;
                score_pos_ = score_neg_ = 0.0;
                auto_dir_on_ = false;
            }
        }

        const float w_phase = std::max(1e-9f, circularGaussianBand(u));
        const float w_vm = std::exp(state_.kappa * (std::cos(del) - 1.f));
        const double R = meas_var_base_ / std::max(1e-3, double(state_.kappa) * double(w_phase));
        const double S = P_(0, 0) + R;
        Eigen::Vector3d K = P_.col(0) / S;
        const float wEKF_vis = float(std::clamp(K(0), 0.0, 1.0));
        phi_now_ -= K(0) * double(del);
        state_.omega -= K(1) * double(del);
        state_.alpha -= K(2) * double(del);
        Eigen::RowVector3d Hrow;
        Hrow << 1.0, 0.0, 0.0;
        P_ = (Eigen::Matrix3d::Identity() - K * Hrow) * P_;
        state_.phi0 = phi_now_ - state_.omega * t - 0.5 * state_.alpha * t * t;

        if (auto_dir_on_) return;
        if (gpu_accum_on_) return;

        Eigen::Vector2f dtheta_du(-float(sgn) * u.y() / (unorm * unorm),
                                  float(sgn) * u.x() / (unorm * unorm));
        Eigen::Vector2f dr_du = u / unorm;

        Eigen::Matrix<float, 2, 6> du_dq;
        Hsnap.du_dparams_analytic(xpix, du_dq);

        // These Jacobians connect the rotor-plane phase and radius residuals
        // back to homography parameters for the batched Gauss-Newton solve.
        Eigen::Matrix<float, 1, 6> Jr_phase =
            -float(B_) * (dtheta_du.transpose() * du_dq) - beta_.deriv(unorm) * (dr_du.transpose() * du_dq);
        Jr_phase(0, 1) += -float(B_);

        const float r_resid = unorm - 1.0f;
        Eigen::Matrix<float, 1, 6> Jr_rad = (dr_du.transpose() * du_dq);

        if (tip_balance_on_) {
            // Balanced Tip Occupancy uses three radial bands around the blade
            // tip: a core band at r = 1 and inside/outside side bands.
            const float r = unorm;

            auto gauss = [](float r_value, float mu, float sig) {
                const float z = (r_value - mu) / sig;
                return std::exp(-0.5f * z * z);
            };
            auto dgauss_dr = [](float r_value, float mu, float sig, float val) {
                return val * (-(r_value - mu) / (sig * sig));
            };

            const float b_core = gauss(r, 1.0f, sigma_core_);
            const float b_in = gauss(r, 1.0f - delta_tip_, sigma_in_);
            const float b_out = gauss(r, 1.0f + delta_tip_, sigma_out_);

            const float db_core_dr = dgauss_dr(r, 1.0f, sigma_core_, b_core);
            const float db_in_dr = dgauss_dr(r, 1.0f - delta_tip_, sigma_in_, b_in);
            const float db_out_dr = dgauss_dr(r, 1.0f + delta_tip_, sigma_out_, b_out);

            const float w_edge = w_vm * w_phase;

            {
                std::lock_guard<std::mutex> lk(accum_mutex_);
                C_core_ += double(w_edge * b_core);
                C_in_ += double(w_edge * b_in);
                C_out_ += double(w_edge * b_out);

                const Eigen::Matrix<double, 6, 1> jt = Jr_rad.transpose().cast<double>();
                J_core_.noalias() += double(w_edge * db_core_dr) * jt;
                J_in_.noalias() += double(w_edge * db_in_dr) * jt;
                J_out_.noalias() += double(w_edge * db_out_dr) * jt;
            }

            const float W_vis = std::max(0.f, w_edge * (b_out - b_in));
            accumulate_event_weights_pixel_(x, y, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, W_vis);
        }

        const float w_rad = std::max(1e-9f, w_phase);

        constexpr float kHuberPhase = 0.6f;
        constexpr float kHuberRad = 0.3f;
        const float wr_phase = huber_weight(del, kHuberPhase);
        const float wr_rad = huber_weight(r_resid, kHuberRad);

        const float wP = w_phase * w_vm * wr_phase;
        const float wR = w_rad * w_vm * wr_rad;

        float r_pol = 0.f;
        Eigen::Matrix<float, 6, 1> Jt_pol;
        Eigen::Matrix<float, 1, 6> J_pol;
        bool have_pol = false;
        if (pol_phase_on_) {
            // The polarity residual softly aligns positive and negative events
            // with the expected blade-edge phase.
            const float gate = std::max(0.f, circularGaussianBand(u, 0.5));
            const float dpol = (polarity >= 0 ? delta_pol_pos_ : delta_pol_neg_);
            const float kappaP = std::max(1e-6f, state_.kappa * kappa_pol_scale_);

            const float del_pol = helixtrack::wrap_pi(del - dpol);
            const float w_vm_pol = std::exp(kappaP * (std::cos(del_pol) - 1.f));

            const float c_pol = std::sqrt(2.f * lambda_pol_ * kappaP);
            r_pol = c_pol * std::sin(0.5f * del_pol) * gate;
            const float scale_pol =
                std::sqrt(0.5f * lambda_pol_ * kappaP) * std::cos(0.5f * del_pol) * gate;
            J_pol = w_vm_pol * scale_pol * Jr_phase;
            Jt_pol = J_pol.transpose();
            accumulate_event_weights_pixel_(x, y, 0.f, 0.f, lambda_pol_ * w_vm_pol * gate, 0.f);
            have_pol = true;
        }

        accumulate_event_weights_pixel_(x, y, lambda_phase_ * wP, lambda_rad_ * wR, 0.f, 0.f, 0.f, wEKF_vis);

        const Eigen::Matrix<float, 6, 1> Jt_phase = Jr_phase.transpose();
        const Eigen::Matrix<float, 6, 1> Jt_rad = Jr_rad.transpose();
        {
            std::lock_guard<std::mutex> lk(accum_mutex_);
            // Accumulate the Gauss-Newton normal equations for the phase and
            // radial terms on the current event batch.
            AtA_.noalias() += double(lambda_phase_ * wP) * (Jt_phase.cast<double>() * Jr_phase.cast<double>());
            Atb_.noalias() += double(lambda_phase_ * wP) * double(del) * Jt_phase.cast<double>();

            AtA_.noalias() += double(lambda_rad_ * wR) * (Jt_rad.cast<double>() * Jr_rad.cast<double>());
            Atb_.noalias() += double(lambda_rad_ * wR) * double(r_resid) * Jt_rad.cast<double>();

            if (have_pol) {
                AtA_.noalias() += (Jt_pol.cast<double>() * J_pol.cast<double>());
                Atb_.noalias() += double(r_pol) * Jt_pol.cast<double>();
            }
            ++batch_count_;
            last_time_s_ = t;
        }

        if (band_barrier_on_) {
            // Soft annulus barriers keep the back-warped events inside the
            // informative rotor band delimited by r_inner and r_outer.
            const float r_in = r_inner_;
            const float r_out = r_outer_;

            const float sp_in = softplus_tau(r_in - unorm, tau_band_);
            const float sp_out = softplus_tau(unorm - r_out, tau_band_);
            const float sig_in = sigmoid((r_in - unorm) / tau_band_);
            const float sig_out = sigmoid((unorm - r_out) / tau_band_);
            const float c_band = std::sqrt(lambda_band_);

            const float r_in_res = c_band * sp_in;
            const float r_out_res = c_band * sp_out;

            Eigen::Matrix<float, 1, 6> J_in = (-c_band * sig_in) * Jr_rad;
            Eigen::Matrix<float, 1, 6> J_out = (c_band * sig_out) * Jr_rad;

            Eigen::Matrix<float, 6, 1> Jt_in = J_in.transpose();
            Eigen::Matrix<float, 6, 1> Jt_out = J_out.transpose();

            {
                std::lock_guard<std::mutex> lk(accum_mutex_);
                AtA_.noalias() += double(w_vm) * (Jt_in.cast<double>() * J_in.cast<double>());
                Atb_.noalias() += double(w_vm) * double(r_in_res) * Jt_in.cast<double>();
                AtA_.noalias() += double(w_vm) * (Jt_out.cast<double>() * J_out.cast<double>());
                Atb_.noalias() += double(w_vm) * double(r_out_res) * Jt_out.cast<double>();
            }

            const float wIn_vis = std::max(0.f, w_vm * (c_band * sig_in));
            const float wOut_vis = std::max(0.f, w_vm * (c_band * sig_out));
            if (wIn_vis > 0.f || wOut_vis > 0.f) {
                accumulate_event_weights_pixel_(x, y, 0.f, 0.f, 0.f, wIn_vis, wOut_vis, 0.f, 0.f);
            }
        }
    }

    void solve_pose_and_reset(float lambda_diag = 1e-3f, std::size_t min_batch = 2000) {
        // Solve the batched Gauss-Newton system and apply per-parameter step
        // sizes for scale, rotation, translation, and perspective.
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();

        {
            std::lock_guard<std::mutex> lk(accum_mutex_);
            if (batch_count_ < min_batch) {
                std::cout << "[HelixTrack] Warning: insufficient events for pose update: "
                          << batch_count_ << " < " << min_batch << std::endl;
                return;
            }
            H = AtA_;
            g = Atb_;
            AtA_ *= rho_;
            Atb_ *= rho_;
            batch_count_ = 0;
        }

        if (tip_balance_on_) {
            // Add the Balanced Tip Occupancy contribution after the per-event
            // residual terms have been accumulated.
            const double Cc = C_core_, Cin = C_in_, Cout = C_out_;
            const Eigen::Matrix<double, 6, 1> Jc = J_core_, Jin = J_in_, Jout = J_out_;
            C_core_ = C_in_ = C_out_ = 0.0;
            J_core_.setZero();
            J_in_.setZero();
            J_out_.setZero();

            const double eps = 1e-9;
            if (Cc > eps && std::isfinite(Cc)) {
                const double p_in = Cin / (Cc + eps);
                const double p_out = Cout / (Cc + eps);

                Eigen::Matrix<double, 1, 6> Jp_out =
                    ((Cc * Jout) - (Cout * Jc)).transpose() / ((Cc + eps) * (Cc + eps));
                Eigen::Matrix<double, 1, 6> Jp_in =
                    ((Cc * Jin) - (Cin * Jc)).transpose() / ((Cc + eps) * (Cc + eps));

                const double c_tip = std::sqrt(double(lambda_tip_));

                const double z_out = (p_out - double(p_out_max_)) / double(tau_occ_);
                const double s_out = 1.0 / (1.0 + std::exp(-z_out));
                const double r_out = c_tip * double(softplus_tau(float(p_out - p_out_max_), tau_occ_));

                Eigen::Matrix<double, 1, 6> J_out_term = c_tip * s_out * Jp_out;
                Eigen::Matrix<double, 6, 1> Jt_out = J_out_term.transpose();
                H.noalias() += (Jt_out * J_out_term);
                g.noalias() += (r_out * Jt_out);

                const double z_in = (double(p_in_min_) - p_in) / double(tau_occ_);
                const double s_in = 1.0 / (1.0 + std::exp(-z_in));
                const double r_in = c_tip * double(softplus_tau(float(p_in_min_ - p_in), tau_occ_));

                Eigen::Matrix<double, 1, 6> J_in_term = c_tip * s_in * (-Jp_in);
                Eigen::Matrix<double, 6, 1> Jt_in = J_in_term.transpose();
                H.noalias() += (Jt_in * J_in_term);
                g.noalias() += (r_in * Jt_in);
            }
        }

        for (int i = 0; i < 6; ++i) H(i, i) += double(lambda_diag);

        const Eigen::Matrix<double, 6, 1> dq_d = -H.ldlt().solve(g);
        if (!dq_d.allFinite()) return;
        Eigen::Matrix<float, 6, 1> dq = dq_d.cast<float>();
        dq(0) *= gn_gamma_scale_;
        dq(1) *= gn_gamma_rotation_;
        dq(2) *= gn_gamma_translation_;
        dq(3) *= gn_gamma_translation_;
        dq(4) *= gn_gamma_perspective_;
        dq(5) *= gn_gamma_perspective_;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            pose_.applyIncrement(dq);
        }
    }

    template <typename Drawer>
    void draw_predicted_ring(Drawer &&draw_pixel, int samples = 720) {
        // Visualize the current helical phase model projected back into the
        // image with the latest homography estimate.
        PoseH Hsnap;
        State Ssnap;
        double t;
        {
            std::lock_guard<std::mutex> lk1(pose_mutex_);
            Hsnap = pose_;
            Ssnap = state_;
        }
        t = last_time_s_;

        const float r0 = 1.0f;
        const int sgn = rotation_sign_;
        const double phi_eff = Ssnap.phase_at(t) - double(B_) * double(Hsnap.q(1));

        for (int k = 0; k < samples; ++k) {
            const float th = kTwoPI * (k / float(samples));
            const Eigen::Vector2f u(r0 * std::cos(th), r0 * std::sin(th));
            const float theta_dir = std::atan2(float(sgn) * u.y(), u.x());
            const float delta_k =
                helixtrack::wrap_pi(float(phi_eff - float(B_) * theta_dir - beta_.eval(r0)));
            const float w = std::exp(Ssnap.kappa * (std::cos(delta_k) - 1.f));
            if (w < 1e-3f) continue;

            const Eigen::Vector2f x = Hsnap.x_from_u(u);
            draw_pixel(int(std::lround(x.x())), int(std::lround(x.y())), w, cv::Vec3f(0.f, 1.f, 0.f));
        }

        if (radial_roi_enabled_) {
            draw_u_circle(draw_pixel, r_inner_, 360, cv::Vec3f(.8f, 0.f, 0.f));
            draw_u_circle(draw_pixel, r_outer_, 360, cv::Vec3f(.8f, .0f, .0f));
        }
    }

    State state() const { return state_; }

    PoseH pose() const {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        return pose_;
    }

    double last_time_s() const { return last_time_s_; }
    int width() const { return width_; }
    int height() const { return height_; }

    int num_events_in_H_batch() const {
        std::lock_guard<std::mutex> lk(accum_mutex_);
        return batch_count_;
    }

    void enable_weights_visualization(bool on = true) {
        heatmaps_.set_enabled(on, width_, height_);
    }

    void draw_event_weights_cv(cv::Mat &frame, float alpha = 0.65f, int mode = 0,
                               int colormap = cv::COLORMAP_VIRIDIS, bool log_scale = true,
                               float clip_sigma = 3.0f) {
        heatmaps_.draw_overlay(frame, alpha, mode, colormap, log_scale, clip_sigma);
    }

private:
    static inline float huber_weight(float r, float c) {
        const float a = std::fabs(r);
        return (a <= c) ? 1.f : (c / a);
    }

    inline void accumulate_event_weights_pixel_(int x, int y, float wP, float wR, float wPol,
                                                float wBarIn = 0.f, float wBarOut = 0.f,
                                                float wEkf = 0.f, float W_half = 0.f) {
        heatmaps_.accumulate(x, y, wP, wR, wPol, wBarIn, wBarOut, wEkf, W_half);
    }

    int width_{0};
    int height_{0};
    mutable std::mutex pose_mutex_;
    PoseH pose_;
    State state_;
    int B_{2};
    BetaTwist beta_{};
    double phi_now_{0.0};
    Eigen::Matrix3d P_;
    double q_jerk_{1};
    double meas_var_base_{1.0};
    double last_kf_time_{-1.0};

    mutable std::mutex accum_mutex_;
    Eigen::Matrix<double, 6, 6> AtA_ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> Atb_ = Eigen::Matrix<double, 6, 1>::Zero();
    std::size_t batch_count_{0};

    bool t0_set_{false};
    std::int64_t t0_us_{0};
    double last_time_s_{0.0};

    bool radial_roi_enabled_{false};
    float r_inner_{0.3f};
    float r_outer_{1.7f};

    WeightHeatmaps heatmaps_;

    std::atomic<int> rotation_sign_{+1};
    bool auto_dir_on_{false};
    std::size_t auto_dir_counter_{0};
    double score_pos_{0.0}, score_neg_{0.0};
    const std::size_t auto_dir_period_{5000};
    const double auto_dir_margin_{0.02};

    bool pol_phase_on_{false};
    float lambda_pol_{1.0f};
    float delta_pol_pos_{+0.20f};
    float delta_pol_neg_{-0.20f};
    float kappa_pol_scale_{1.0f};

    bool band_barrier_on_{false};
    float lambda_band_{10.0f};
    float tau_band_{0.03f};

    float lambda_phase_{1.0f};
    float lambda_rad_{1.0f};
    float rho_{0.0f};

    bool tip_balance_on_{false};
    float lambda_tip_{0.f};
    float sigma_core_{0.06f};
    float delta_tip_{0.03f};
    float sigma_in_{0.02f};
    float sigma_out_{0.02f};
    float tau_occ_{0.05f};
    float p_in_min_{0.50f};
    float p_out_max_{0.50f};

    float gn_gamma_scale_{1.0f};
    float gn_gamma_rotation_{1.0f};
    float gn_gamma_translation_{1.0f};
    float gn_gamma_perspective_{1.0f};

    double C_core_{0.0}, C_in_{0.0}, C_out_{0.0};
    Eigen::Matrix<double, 6, 1> J_core_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> J_in_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> J_out_ = Eigen::Matrix<double, 6, 1>::Zero();

    bool gpu_accum_on_{false};
};

}  // namespace helixtrack

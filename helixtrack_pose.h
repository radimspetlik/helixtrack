#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "helixtrack_math.h"

namespace helixtrack {

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Mat2f = Eigen::Matrix2f;
using Mat3f = Eigen::Matrix3f;
using Mat26f = Eigen::Matrix<float, 2, 6>;

struct BetaTwist {
    float b0{0.f};
    float b1{0.f};
    float b2{0.f};

    float eval(float r) const { return b0 + b1 * r + b2 * r * r; }
    float deriv(float r) const { return b1 + 2.f * b2 * r; }
};

// PoseH stores the image-to-rotor-plane homography, the inverse back-warp used
// to lift events into rotor coordinates, and the compact 6-DoF pose parameters
// optimized by the batched pose refinement stage.
struct PoseH {
    Vec6f q;
    Mat3f H;
    Mat3f Hinv;

    PoseH() {
        q.setZero();
        rebuild();
    }

    void rebuild() {
        const float s = std::exp(q(0));
        const float c = std::cos(q(1));
        const float sn = std::sin(q(1));

        Mat3f homography = Mat3f::Identity();
        homography(0, 0) = s * c;
        homography(0, 1) = -s * sn;
        homography(1, 0) = s * sn;
        homography(1, 1) = s * c;
        homography(0, 2) = q(2);
        homography(1, 2) = q(3);
        homography(2, 0) = q(4);
        homography(2, 1) = q(5);

        H = homography;
        Hinv = H.inverse();
    }

    static float rho_from_aspect(float aspect_raw) {
        float aspect = std::max(1e-6f, aspect_raw);
        if (aspect < 1.f) aspect = 1.f / aspect;
        const float rho = (aspect - 1.f) / (aspect + 1.f);
        return std::clamp(rho, 0.f, 0.95f);
    }

    static float aspect_from_rho(float rho_raw) {
        const float rho = std::clamp(std::abs(rho_raw), 0.f, 0.999f);
        return (1.f + rho) / (1.f - rho);
    }

    float unit_circle_aspect() const {
        const float rho = std::sqrt(q(4) * q(4) + q(5) * q(5));
        return aspect_from_rho(rho);
    }

    void set_perspective_from_aspect(float aspect, float dir_angle_u_rad = 0.f) {
        const float rho = rho_from_aspect(aspect);
        q(4) = rho * std::cos(dir_angle_u_rad);
        q(5) = rho * std::sin(dir_angle_u_rad);
        rebuild();
    }

    void setParams(const Vec6f &new_q) {
        q = new_q;
        rebuild();
    }

    void applyIncrement(const Vec6f &delta_q) {
        q += delta_q;
        rebuild();
    }

    void setSimilarity(float scale, float psi, const Vec2f &translation) {
        q.setZero();
        q(0) = std::log(std::max(1e-6f, scale));
        q(1) = psi;
        q(2) = translation.x();
        q(3) = translation.y();
        rebuild();
    }

    void setSimilarityWithAspect(float scale, float psi, const Vec2f &translation, float aspect) {
        q.setZero();
        q(0) = std::log(std::max(1e-6f, scale));
        q(1) = psi;
        q(2) = translation.x();
        q(3) = translation.y();
        q(4) = rho_from_aspect(aspect);
        q(5) = 0.f;
        rebuild();
    }

    Vec2f x_from_u(const Vec2f &u) const {
        const Vec3f up(u.x(), u.y(), 1.f);
        const Vec3f xp = H * up;
        const float w = std::max(1e-6f, xp.z());
        return {xp.x() / w, xp.y() / w};
    }

    Vec2f u_from_x(const Vec2f &x) const {
        const Vec3f xp(x.x(), x.y(), 1.f);
        const Vec3f up = Hinv * xp;
        const float w = std::max(1e-6f, up.z());
        return {up.x() / w, up.y() / w};
    }

    Mat2f dx_du_at_u(const Vec2f &u) const {
        const float s = std::max(1e-6f, H(2, 0) * u.x() + H(2, 1) * u.y() + 1.f);
        const Vec3f up(u.x(), u.y(), 1.f);
        const Vec3f xp = H * up;
        const float x = xp.x() / s;
        const float y = xp.y() / s;

        Mat2f jacobian;
        jacobian << (H(0, 0) - H(2, 0) * x) / s, (H(0, 1) - H(2, 1) * x) / s,
                    (H(1, 0) - H(2, 0) * y) / s, (H(1, 1) - H(2, 1) * y) / s;
        return jacobian;
    }

    // Analytic d u / d q Jacobian for the back-warp. This is the core geometry
    // derivative used to turn phase, radial, polarity, and band residuals into
    // a Gauss-Newton system over homography parameters.
    void du_dparams_analytic(const Vec2f &x, Mat26f &J, float /*base_eps*/ = 1e-3f) const {
        const Mat3f &K = Hinv;
        const Vec3f xp(x.x(), x.y(), 1.f);
        const Vec3f up = K * xp;

        const float w = std::max(1e-6f, up.z());
        const float inv_w = 1.f / w;
        const float ux = up.x() * inv_w;
        const float uy = up.y() * inv_w;

        const float s = std::exp(q(0));
        const float c = std::cos(q(1));
        const float sn = std::sin(q(1));

        const float k00 = K(0, 0);
        const float k01 = K(0, 1);
        const float k02 = K(0, 2);
        const float k10 = K(1, 0);
        const float k11 = K(1, 1);
        const float k12 = K(1, 2);
        const float k20 = K(2, 0);
        const float k21 = K(2, 1);
        const float k22 = K(2, 2);

        auto push = [&](int col, float vx, float vy, float vz) {
            const float d0 = -(k00 * vx + k01 * vy + k02 * vz);
            const float d1 = -(k10 * vx + k11 * vy + k12 * vz);
            const float d2 = -(k20 * vx + k21 * vy + k22 * vz);
            J(0, col) = inv_w * (d0 - ux * d2);
            J(1, col) = inv_w * (d1 - uy * d2);
        };

        push(0, s * c * up.x() + (-s * sn) * up.y(), s * sn * up.x() + s * c * up.y(), 0.f);
        push(1, -s * sn * up.x() + (-s * c) * up.y(), s * c * up.x() + (-s * sn) * up.y(), 0.f);
        push(2, w, 0.f, 0.f);
        push(3, 0.f, w, 0.f);
        push(4, 0.f, 0.f, up.x());
        push(5, 0.f, 0.f, up.y());
    }

    void du_dparams_analyticslow(const Vec2f &x, Mat26f &J, float /*base_eps*/ = 1e-3f) const {
        const Vec3f xp(x.x(), x.y(), 1.f);
        const Mat3f &K = Hinv;
        const Vec3f up = K * xp;

        const float w = std::max(1e-6f, up.z());
        const float inv_w = 1.f / w;
        const float ux = up.x() * inv_w;
        const float uy = up.y() * inv_w;

        auto push_col = [&](int k, const Mat3f &dH) {
            const Vec3f dup = -K * (dH * up);
            const float d0 = dup.x();
            const float d1 = dup.y();
            const float d2 = dup.z();
            J(0, k) = inv_w * (d0 - ux * d2);
            J(1, k) = inv_w * (d1 - uy * d2);
        };

        const float s = std::exp(q(0));
        const float c = std::cos(q(1));
        const float sn = std::sin(q(1));

        {
            Mat3f dH = Mat3f::Zero();
            dH(0, 0) = s * c;
            dH(0, 1) = -s * sn;
            dH(1, 0) = s * sn;
            dH(1, 1) = s * c;
            push_col(0, dH);
        }
        {
            Mat3f dH = Mat3f::Zero();
            dH(0, 0) = -s * sn;
            dH(0, 1) = -s * c;
            dH(1, 0) = s * c;
            dH(1, 1) = -s * sn;
            push_col(1, dH);
        }
        {
            Mat3f dH = Mat3f::Zero();
            dH(0, 2) = 1.f;
            push_col(2, dH);
        }
        {
            Mat3f dH = Mat3f::Zero();
            dH(1, 2) = 1.f;
            push_col(3, dH);
        }
        {
            Mat3f dH = Mat3f::Zero();
            dH(2, 0) = 1.f;
            push_col(4, dH);
        }
        {
            Mat3f dH = Mat3f::Zero();
            dH(2, 1) = 1.f;
            push_col(5, dH);
        }
    }

    void du_dparams_numeric(const Vec2f &x, Mat26f &J, float base_eps = 1e-3f) const {
        PoseH tmp = *this;
        const Vec2f u0 = u_from_x(x);

        for (int k = 0; k < 6; ++k) {
            float scale = 1.f;
            if (k == 2 || k == 3) scale = std::max(1.f, std::abs(q(k)));
            if (k == 4 || k == 5) scale = 1e-3f;
            const float eps = base_eps * scale;

            Vec6f qp = q;
            qp(k) += eps;
            tmp.setParams(qp);
            const Vec2f u1 = tmp.u_from_x(x);
            J.col(k) = (u1 - u0) / eps;
        }
    }
};

struct State {
    double phi0{0.0};
    double omega{rpm_to_rad_per_sec(100.0)};
    double alpha{0.0};
    float kappa{20.f};

    // The phase tracker models phase, angular speed, and angular acceleration
    // directly in continuous time.
    double phase_at(double t) const {
        return phi0 + omega * t + 0.5 * alpha * t * t;
    }
};

}  // namespace helixtrack

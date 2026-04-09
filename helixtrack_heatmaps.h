#pragma once

#include <array>
#include <atomic>
#include <mutex>

#include <opencv2/opencv.hpp>

namespace helixtrack {

class WeightHeatmaps {
public:
    void set_enabled(bool enabled, int width, int height) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_.store(enabled, std::memory_order_relaxed);
        width_ = width;
        height_ = height;

        if (enabled) {
            for (auto &buffer : buffers_) {
                buffer.create(height_, width_);
            }
            active_idx_ = 0;
        } else {
            for (auto &buffer : buffers_) {
                buffer.release();
            }
            active_idx_ = 0;
        }
    }

    bool enabled() const {
        return enabled_.load(std::memory_order_relaxed);
    }

    void accumulate(int x, int y, float w_phase, float w_rad, float w_pol, float w_bar_in = 0.f,
                    float w_bar_out = 0.f, float w_ekf = 0.f, float w_half = 0.f) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_.load(std::memory_order_relaxed)) return;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return;

        BufferSet &buffer = buffers_[active_idx_];
        buffer.phase.at<float>(y, x) += w_phase;
        buffer.rad.at<float>(y, x) += w_rad;
        buffer.pol.at<float>(y, x) += w_pol;
        buffer.bar_in.at<float>(y, x) += w_bar_in;
        buffer.bar_out.at<float>(y, x) += w_bar_out;
        buffer.ekf.at<float>(y, x) += w_ekf;
        buffer.half.at<float>(y, x) += w_half;
    }

    void draw_overlay(cv::Mat &frame, float alpha = 0.65f, int mode = 0,
                      int colormap = cv::COLORMAP_VIRIDIS, bool log_scale = true,
                      float clip_sigma = 3.0f) {
        if (!enabled()) return;

        BufferSet snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!enabled_.load(std::memory_order_relaxed)) return;

            const int read_idx = active_idx_;
            active_idx_ = 1 - active_idx_;
            snapshot = buffers_[read_idx].clone();
            buffers_[read_idx].clear();
        }

        cv::Mat src;
        switch (mode) {
            case 1:
                src = snapshot.phase;
                break;
            case 2:
                src = snapshot.rad;
                break;
            case 3:
                src = snapshot.pol;
                break;
            case 4:
                src = snapshot.bar_in;
                break;
            case 5:
                src = snapshot.bar_out;
                break;
            case 6:
                src = snapshot.ekf;
                break;
            case 7:
                src = snapshot.half;
                break;
            default:
                src = snapshot.phase + snapshot.rad + snapshot.pol + snapshot.bar_in +
                      snapshot.bar_out + snapshot.ekf + snapshot.half;
                break;
        }
        if (src.empty()) return;

        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(src, mean, stddev);
        const double max_clip = std::max(1e-6, mean[0] + clip_sigma * stddev[0]);

        cv::Mat normf;
        if (log_scale) {
            const double gain = 4.0;
            cv::Mat scaled = src / max_clip;
            cv::min(scaled, 1.0, scaled);
            cv::log(1.0 + gain * scaled, normf);
            normf /= std::log(1.0 + gain);
        } else {
            normf = src / max_clip;
            cv::min(normf, 1.0, normf);
        }

        cv::Mat u8;
        cv::Mat color;
        normf.convertTo(u8, CV_8U, 255.0);
        cv::applyColorMap(u8, color, colormap);
        cv::addWeighted(color, alpha, frame, 1.0f - alpha, 0.0, frame);
    }

private:
    struct BufferSet {
        cv::Mat phase;
        cv::Mat rad;
        cv::Mat pol;
        cv::Mat bar_in;
        cv::Mat bar_out;
        cv::Mat ekf;
        cv::Mat half;

        void create(int height, int width) {
            phase.create(height, width, CV_32F);
            rad.create(height, width, CV_32F);
            pol.create(height, width, CV_32F);
            bar_in.create(height, width, CV_32F);
            bar_out.create(height, width, CV_32F);
            ekf.create(height, width, CV_32F);
            half.create(height, width, CV_32F);
            clear();
        }

        void clear() {
            if (!phase.empty()) phase.setTo(0);
            if (!rad.empty()) rad.setTo(0);
            if (!pol.empty()) pol.setTo(0);
            if (!bar_in.empty()) bar_in.setTo(0);
            if (!bar_out.empty()) bar_out.setTo(0);
            if (!ekf.empty()) ekf.setTo(0);
            if (!half.empty()) half.setTo(0);
        }

        void release() {
            phase.release();
            rad.release();
            pol.release();
            bar_in.release();
            bar_out.release();
            ekf.release();
            half.release();
        }

        BufferSet clone() const {
            BufferSet copy;
            copy.phase = phase.clone();
            copy.rad = rad.clone();
            copy.pol = pol.clone();
            copy.bar_in = bar_in.clone();
            copy.bar_out = bar_out.clone();
            copy.ekf = ekf.clone();
            copy.half = half.clone();
            return copy;
        }
    };

    std::atomic<bool> enabled_{false};
    int width_{0};
    int height_{0};
    mutable std::mutex mutex_;
    std::array<BufferSet, 2> buffers_{};
    int active_idx_{0};
};

}  // namespace helixtrack

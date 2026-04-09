#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hdf_utils.h"

template <typename T>
inline double mean_of(const std::vector<T> &values) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    long double sum = 0;
    for (const auto &value : values) sum += static_cast<long double>(value);
    return static_cast<double>(sum / static_cast<long double>(values.size()));
}

template <typename T>
inline double std_of(const std::vector<T> &values, double mean) {
    if (values.size() < 2) return 0.0;
    long double sum_sq = 0;
    for (const auto &value : values) {
        const long double diff = static_cast<long double>(value) - mean;
        sum_sq += diff * diff;
    }
    return std::sqrt(static_cast<double>(sum_sq / static_cast<long double>(values.size() - 1)));
}

template <typename T>
inline T percentile(std::vector<T> values, double p01) {
    if (values.empty()) return T(0);
    if (p01 <= 0.0) return *std::min_element(values.begin(), values.end());
    if (p01 >= 1.0) return *std::max_element(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(std::floor(p01 * (values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

inline void write_scalar_1d(H5::Group &group, const std::string &name, double value) {
    std::vector<double> values{value};
    std::string dataset_name = name;
    write_results_to_hdf5(group, dataset_name, values);
}

inline void write_scalar_1d(H5::Group &group, const std::string &name, long long value) {
    std::vector<long long> values{value};
    std::string dataset_name = name;
    write_results_to_hdf5(group, dataset_name, values);
}

inline double round_to(double value, double quantum) {
    return std::round(value / quantum) * quantum;
}

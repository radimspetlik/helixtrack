#pragma once

#include <string>
#include <vector>

#include "H5Cpp.h"

inline void write_results_to_hdf5(H5::Group &channel_group, std::string &dataset_name,
                                  const std::vector<long long> &data) {
    if (channel_group.exists(dataset_name)) {
        channel_group.unlink(dataset_name);
    }
    hsize_t vector_size = data.size();
    H5::DataSpace dataspace(1, &vector_size);
    auto dataset = channel_group.createDataSet(dataset_name, H5::PredType::NATIVE_LLONG, dataspace);
    dataset.write(data.data(), H5::PredType::NATIVE_LLONG);
}

inline void write_results_to_hdf5(H5::Group &channel_group, std::string &dataset_name,
                                  const std::vector<double> &data) {
    if (channel_group.exists(dataset_name)) {
        channel_group.unlink(dataset_name);
    }
    hsize_t vector_size = data.size();
    H5::DataSpace dataspace(1, &vector_size);
    auto dataset = channel_group.createDataSet(dataset_name, H5::PredType::NATIVE_DOUBLE, dataspace);
    dataset.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

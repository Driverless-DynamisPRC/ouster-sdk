#pragma once

#include <cstdint>

#include "ouster/types.h"


namespace ouster {

inline size_t get_expected_packets(const size_t w, const sensor::packet_format &pf) {
    return w / pf.columns_per_packet + (w % pf.columns_per_packet ? 1 : 0);
}

inline size_t get_expected_packets(const sensor::sensor_info& info,
                                   const sensor::packet_format& pf,
                                   const size_t max_packets) {
    size_t expected_packets;

    if (info.format.column_window.second < info.format.column_window.first) {
        // the valid azimuth window wraps through 0
        int start_packet =
            info.format.column_window.second / pf.columns_per_packet;
        int end_packet =
            info.format.column_window.first / pf.columns_per_packet;
        expected_packets = start_packet + 1 + (max_packets - end_packet);
        // subtract one if start and end are in the same block
        if (start_packet == end_packet) {
            expected_packets -= 1;
        }
    } else {
        // no wrapping of azimuth the window through 0
        int start_packet =
            info.format.column_window.first / pf.columns_per_packet;
        int end_packet =
            info.format.column_window.second / pf.columns_per_packet;

        expected_packets = end_packet - start_packet + 1;
    }

    return expected_packets;
}

inline size_t get_expected_packets(const sensor::sensor_info &info) {
    const auto pf = sensor::get_format(info);
    const auto max_packets = get_expected_packets(
        info.format.columns_per_frame, pf);
    return get_expected_packets(info, pf, max_packets);
}

} // namespace ouster

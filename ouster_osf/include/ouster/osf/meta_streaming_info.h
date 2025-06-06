/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file meta_streaming_info.h
 * @brief Metadata entry StreamingInfo
 *
 */
#pragma once

#include <iostream>
#include <memory>

#include "ouster/osf/metadata.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

/**
 * Class for keeping track of OSF chunks.
 *
 * Flat Buffer Reference:
 *   fb/streaming/streaming_info.fbs :: ChunkInfo
 */
struct OUSTER_API_CLASS ChunkInfo {
    /**
     * The offset in the flatbuffer where
     * the chunk is located.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: offset
     */
    uint64_t offset;

    /**
     * The specific stream the chunk is associated with.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: stream_id
     */
    uint32_t stream_id;

    /**
     * The number of messages in the chunk
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: message_count
     */
    uint32_t message_count;
};

/**
 * Class for keeping track of OSF stream stats.
 *
 * Flat Buffer Reference:
 *   fb/streaming/streaming_info.fbs :: StreamStats
 */
struct OUSTER_API_CLASS StreamStats {
    /**
     * The specific stream the chunk is associated with.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: stream_id
     */
    uint32_t stream_id;

    /**
     * The first timestamp in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: start_ts
     */
    ts_t start_ts;

    /**
     * The last timestamp in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: end_ts
     */
    ts_t end_ts;

    /**
     * The number of messages in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: message_count
     */
    uint64_t message_count;

    /**
     * The average size of the messages in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: message_avg_size
     */
    uint32_t message_avg_size;

    /**
     * The receive timestamps of each message in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: receive_timestamps
     */
    std::vector<uint64_t> receive_timestamps;

    /**
     * The sensor timestamps of each message in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: sensor_timestamps
     */
    std::vector<uint64_t> sensor_timestamps;

    /**
     * Default constructor, sets everthing to 0.
     */
    OUSTER_API_FUNCTION
    StreamStats() = default;

    /**
     * Construct a StreamStats with the specified values
     *
     * @param[in] s_id Specify the stream_id to use.
     * @param[in] receive_ts Set the start and end timestamps to the specified
     * value and add it to the receive timestamps.
     * @param[in] sensor_ts Add to the sensor timestamps.
     * @param[in] msg_size Set the average message size to the specified value.
     */
    OUSTER_API_FUNCTION
    StreamStats(uint32_t s_id, ts_t receive_ts, ts_t sensor_ts,
                uint32_t msg_size);

    /**
     * Update values within the StreamStats
     *
     * @param[in] receive_ts Add another receive timestamp and calculate the
     * start and end values.
     * @param[in] sensor_ts Add another sensor timestamp
     * @param[in] msg_size Add another message size and calculate the average.
     */
    OUSTER_API_FUNCTION
    void update(ts_t receive_ts, ts_t sensor_ts, uint32_t msg_size);
};

/**
 * Get the string representation for a ChunkInfo object.
 *
 * @param[in] chunk_info ChunkInfo object to be converted to string
 *
 * @return The string representation for a ChunkInfo object.
 */
OUSTER_API_FUNCTION
std::string to_string(const ChunkInfo& chunk_info);

/**
 * Get the string representation for a StreamStats object.
 *
 * @param[in] stream_stats StreamStats object to be converted to string.
 *
 * @return The string representation for a StreamStats object.
 */
OUSTER_API_FUNCTION
std::string to_string(const StreamStats& stream_stats);

/**
 * Metadata entry to store StreamingInfo, to support StreamingLayout (RFC 0018)
 *
 * OSF type:
 *   ouster/v1/streaming/StreamingInfo
 *
 * Flat Buffer Reference:
 *   fb/streaming/streaming_info.fbs :: StreamingInfo
 */
class OUSTER_API_CLASS StreamingInfo
    : public MetadataEntryHelper<StreamingInfo> {
   public:
    OUSTER_API_FUNCTION
    StreamingInfo() {}

    /**
     * @param[in] chunks_info Vector containing pairs of
     *                        stream_id/ChunkInfo
     *                        to be used to generate a stream_id/ChunkInfo
     *                        map.
     * @param[in] stream_stats Vector containing pairs of
     *                         stream_id/StreamStats
     *                         to be used to generate a
     *                         stream_id/StreamStats map.
     */
    OUSTER_API_FUNCTION
    StreamingInfo(
        const std::vector<std::pair<uint64_t, ChunkInfo>>& chunks_info,
        const std::vector<std::pair<uint32_t, StreamStats>>& stream_stats);

    /**
     * @param[in] chunks_info ///< Map containing stream_id/ChunkInfo data.
     * @param[in] stream_stats ///< Map containing stream_id/StreamStats data.
     */
    OUSTER_API_FUNCTION
    StreamingInfo(const std::map<uint64_t, ChunkInfo>& chunks_info,
                  const std::map<uint32_t, StreamStats>& stream_stats);

    /**
     * Return the chunk_info map. stream_id/ChunkInfo data.
     *
     * @return The chunk_info map. stream_id/ChunkInfo data.
     */
    OUSTER_API_FUNCTION
    std::map<uint64_t, ChunkInfo>& chunks_info();

    /**
     * Return the stream stat map. stream_id/StreamStats data.
     *
     * @return The stream stat map. stream_id/StreamStats data.
     */
    OUSTER_API_FUNCTION
    std::map<uint32_t, StreamStats>& stream_stats();

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const override final;

    /**
     * Create a StreamingInfo object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new StreamingInfo cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    /**
     * Get the string representation for the LidarSensor object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the LidarSensor object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;

   private:
    /**
     * The internal stream_id to ChunkInfo map.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamingInfo :: chunks
     */
    std::map<uint64_t, ChunkInfo> chunks_info_{};

    /**
     * The internal stream_id to StreamStats map.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamingInfo :: stream_stats
     */
    std::map<uint32_t, StreamStats> stream_stats_{};
};

/** @defgroup OSFTraitsStreamingInfo Templated struct for traits. */
/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsStreamingInfo
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<StreamingInfo> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/streaming/StreamingInfo".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/streaming/StreamingInfo";
    }
};

}  // namespace osf
}  // namespace ouster

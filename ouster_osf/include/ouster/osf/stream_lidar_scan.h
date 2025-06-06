/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file stream_lidar_scan.h
 * @brief Stream of LidarScan
 *
 */
#pragma once

#include "ouster/osf/basics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/writer.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

/**
 * Cast `ls_src` LidarScan to a subset of fields with possible different
 * underlying ChanFieldTypes.
 *
 * @throws std::logic_error Exception on trying to slice a scan with only
 *                          a subset of the requested scans
 *
 * @param[in] ls_src The LidarScan to cast.
 * @param[in] field_types The field types to cast the LidarScan to.
 * @return a copy of `ls_src` with transformed fields.
 */
OUSTER_API_FUNCTION
LidarScan slice_with_cast(const LidarScan& ls_src,
                          const ouster::LidarScanFieldTypes& field_types);

/**
 * Metadata entry for LidarScanStream to store reference to a sensor and
 * field_types
 *
 * OSF type:
 *   ouster/v1/os_sensor/LidarScanStream
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/lidar_scan_stream.fbs
 */
class OUSTER_API_CLASS LidarScanStreamMeta
    : public MetadataEntryHelper<LidarScanStreamMeta> {
   public:
    /**
     * @param[in] sensor_meta_id Reference to LidarSensor metadata that
     *                           describes the sensor configuration.
     * @param[in] field_types LidarScan fields specs, this argument is optional.
     */
    OUSTER_API_FUNCTION
    LidarScanStreamMeta(const uint32_t sensor_meta_id,
                        const ouster::LidarScanFieldTypes field_types = {});

    /**
     * Return the sensor meta id.
     *
     * @return The sensor meta id.
     */
    OUSTER_API_FUNCTION
    uint32_t sensor_meta_id() const;

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a LidarScanStreamMeta object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new LidarScanStreamMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    /**
     * Get the string representation for the LidarScanStreamMeta object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the LidarScanStreamMeta object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;

   private:
    /**
     * Internal store of the sensor id.
     *
     * Flat Buffer Reference:
     *   fb/os_sensor/lidar_scan_stream.fbs :: LidarScanStream :: sensor_id
     */
    uint32_t sensor_meta_id_{0};

    /**
     * Internal store of the field types.
     *
     * Flat Buffer Reference:
     *   fb/os_sensor/lidar_scan_stream.fbs :: LidarScanStream :: field_types
     */
    ouster::LidarScanFieldTypes field_types_;
};

/** @defgroup OSFTraitsLidarScanStreamMeta Templated struct for traits.*/

/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsLidarScanStreamMeta
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<LidarScanStreamMeta> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/LidarScanStream".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/LidarScanStream";
    }
};

/**
 * LidarScanStream that encodes LidarScan objects into the messages.
 *
 * Object type: ouster::sensor::LidarScan
 * Meta type: LidarScanStreamMeta (sensor_meta_id, field_types)
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_scan_stream.fbs
 */
class OUSTER_API_CLASS LidarScanStream
    : public MessageStream<LidarScanStreamMeta, LidarScan> {
   protected:
    friend class Writer;
    friend class MessageRef;

    // Access key pattern used to only allow friends to call our constructor
    struct Token {};

    /**
     * Saves the object to the writer applying the coding/serizlization
     * algorithm defined in make_msg() function. The function is the same for
     * all streams types ...
     *
     * @todo [pb]: Probably should be abstracted/extracted from all streams
     * we also might want to have the corresponding function to read back
     * sequentially from Stream that doesn't seem like fit into this model...
     *
     * @param[in] receive_ts The receive timestamp to use for the lidar scan.
     * @param[in] sensor_ts The sensor timestamp to use for the lidar scan.
     * @param[in] lidar_scan The lidar scan to write.
     */
    void save(const ouster::osf::ts_t receive_ts,
              const ouster::osf::ts_t sensor_ts, const obj_type& lidar_scan);

    flatbuffers::Offset<gen::Field> create_osf_field(
        flatbuffers::FlatBufferBuilder& fbb, const std::string& name,
        const Field& f) const;

    flatbuffers::Offset<gen::LidarScanMsg> create_lidar_scan_msg(
        flatbuffers::FlatBufferBuilder& fbb, const LidarScan& lidar_scan,
        const ouster::sensor::sensor_info& info,
        const ouster::LidarScanFieldTypes meta_field_types) const;

    void fieldEncodeMulti(const LidarScan& lidar_scan,
                          const LidarScanFieldTypes& field_types,
                          const std::vector<int>& px_offset,
                          ScanData& scan_data,
                          const std::vector<size_t>& scan_idxs) const;

    ScanData scanEncodeFieldsSingleThread(
        const LidarScan& lidar_scan, const std::vector<int>& px_offset,
        const LidarScanFieldTypes& field_types) const;

    ScanData scanEncodeFields(const LidarScan& lidar_scan,
                              const std::vector<int>& px_offset,
                              const LidarScanFieldTypes& field_types) const;

    ScanData scanEncode(const LidarScan& lidar_scan,
                        const std::vector<int>& px_offset,
                        const ouster::LidarScanFieldTypes& field_types) const;
    /**
     * Encode/serialize the object to the buffer of bytes.
     *
     * @param[in] lidar_scan The lidar scan to turn into a vector of bytes.
     * @return The byte vector representation of lidar_scan.
     */
    std::vector<uint8_t> make_msg(const obj_type& lidar_scan);

    /**
     * Decode/deserialize the object from bytes buffer using the concrete
     * metadata type for the stream.
     *
     * @param[in] buf The buffer to decode into an object.
     * @param[in] meta The concrete metadata type to use for decoding.
     * @param[in] meta_provider Used to reconstruct any references to other
     *                          metadata entries dependencies
     *                          (like sensor_meta_id)
     * @param[in] fields List of fields to decode. All are decoded if none
     *                   provided.
     * @return Pointer to the decoded object.
     */
    static std::unique_ptr<obj_type> decode_msg(
        const std::vector<uint8_t>& buf, const meta_type& meta,
        const MetadataStore& meta_provider,
        const std::vector<std::string>& fields = {});

   public:
    /**
     * @param[in] key Private class used to prevent non-friends from calling
     * this.
     * @param[in] writer The writer object to use to write messages out.
     * @param[in] sensor_meta_id The sensor to use.
     * @param[in] field_types LidarScan fields specs, this argument is optional.
     */
    OUSTER_API_FUNCTION
    LidarScanStream(Token key, Writer& writer, const uint32_t sensor_meta_id,
                    const ouster::LidarScanFieldTypes& field_types = {});

    /**
     * Return the concrete metadata type.
     * This has templated types.
     *
     * @return The concrete metadata type.
     */
    OUSTER_API_FUNCTION
    const meta_type& meta() const { return meta_; };

   private:
    /**
     * The internal writer object to use to write messages out.
     */
    Writer& writer_;

    /**
     * The internal concrete metadata type.
     */
    meta_type meta_;

    /**
     * The internal flatbuffer id for the stream.
     */
    uint32_t stream_meta_id_{0};

    /**
     * The internal flatbuffer id for the metadata.
     */
    uint32_t sensor_meta_id_{0};

    /**
     * The internal sensor_info data.
     */
    sensor::sensor_info sensor_info_;

    /**
     * The internal field_types data.
     */
    ouster::LidarScanFieldTypes field_types_;
};

}  // namespace osf
}  // namespace ouster

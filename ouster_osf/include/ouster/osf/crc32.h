/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file crc32.h
 * @brief crc32 utility
 *
 */
#pragma once

#include <stddef.h>

#include <cstdint>

#include "ouster/visibility.h"

namespace ouster {
namespace osf {

/**
 *  Size of the CRC field in a buffer
 */
const uint32_t CRC_BYTES_SIZE = 4;

/** @defgroup OsfCRCFunctions Osf CRC Functions. */

/**
 * Caclulate CRC value for the buffer of given size. (ZLIB version)
 *
 * @ingroup OsfCRCFunctions
 *
 * @param[in] buf Pointer to the data buffer.
 * @param[in] size Size of the buffer in bytes.
 * @return CRC32 value
 */
OUSTER_API_FUNCTION
uint32_t crc32(const uint8_t* buf, uint32_t size);

/**
 * Caclulate and append CRC value for the buffer of given size and append
 * it to the initial crc value. (ZLIB version)
 *
 * @ingroup OsfCRCFunctions
 *
 * @param[in] initial_crc Initial crc value to append to.
 * @param[in] buf Pointer to the data buffer.
 * @param[in] size Size of the buffer in bytes.
 * @return CRC32 value
 */
OUSTER_API_FUNCTION
uint32_t crc32(uint32_t initial_crc, const uint8_t* buf, uint32_t size);

}  // namespace osf
}  // namespace ouster

/*
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file calibration_private.h
 *
 * Internal helpers shared between the parts of the calibration library.
 * Not a part of the public API.
 */
#ifndef __CALIBRATION_PRIVATE_H__
#define __CALIBRATION_PRIVATE_H__

#include "calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check whether the method interpolates the raw value logarithmically
 */
bool calibration_log_code(calibration_method_t type);

/**
 * @brief Check whether the method interpolates the calibrated value logarithmically
 */
bool calibration_log_value(calibration_method_t type);

/**
 * @brief Compute the derivatives of the monotone interpolation
 *
 * Allocates the slope array if it does not exist yet and fills it for the
 * current point list.
 *
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_STATE` if there are less than
 * three points, `ESP_ERR_NO_MEM` if the array could not be allocated
 */
esp_err_t calibration_pchip_prepare(calibration_handle_t *handler);

/**
 * @brief Evaluate the monotone interpolation
 *
 * The raw value is transformed logarithmically when needed. Values outside
 * the range of the points are continued linearly with the slope of the
 * outermost segment.
 */
float calibration_pchip_get_value(const calibration_handle_t *handler, float code);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_PRIVATE_H__ */

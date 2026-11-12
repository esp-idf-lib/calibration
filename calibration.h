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
 * @file calibration.h
 * @defgroup calibration calibration
 * @{
 *
 * ESP-IDF Multi-point calibration (approximation) library
 *
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include <esp_err.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Approximation methods
 */
typedef enum
{
    CALIBRATION_LINEAR = 0, //!< Fast linear approximation. The more points, the more accurate approximation
} calibration_method_t;

/**
 * Calibration point
 */
typedef struct
{
    float code;   //!< Raw value
    float value;  //!< Calibrated value
} calibration_point_t;

/**
 * Calibration handler
 */
typedef struct
{
    calibration_method_t type;   //!< Approximation method
    calibration_point_t *points; //!< Ordered list of calibration points
    size_t count;                //!< Maximum number of calibration points
    size_t filled;               //!< Current number of calibration points
} calibration_handle_t;

/**
 * @brief Init calibration handle
 *
 * Allocates memory to store calibration points and fills the handle.
 * The handle must be zeroed before the first call, e.g.
 * `calibration_handle_t handler = { 0 };`.
 *
 * @param handler Pointer to calibration handle structure
 * @param count   Maximum number of calibration points, at least 2
 * @param type    Approximation type
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler is NULL or @p count is less than 2
 *      - `ESP_ERR_NOT_SUPPORTED` if @p type is not supported
 *      - `ESP_ERR_INVALID_STATE` if the handle is already initialized
 *      - `ESP_ERR_NO_MEM` if the point list could not be allocated
 */
esp_err_t calibration_init(calibration_handle_t *handler, size_t count, calibration_method_t type);

/**
 * @brief Add calibration point
 *
 * The point is inserted into the list, which is kept ordered by the raw
 * value. If a point with the same raw value already exists, its calibrated
 * value is replaced. Both values must be finite: NaN and infinities are
 * rejected, because they break the ordering of the list.
 *
 * @param handler Pointer to calibration handle structure
 * @param code    Raw value
 * @param value   Calibrated value
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler is NULL, the handle is not
 *        initialized, or @p code or @p value is not finite
 *      - `ESP_ERR_INVALID_STATE` if the point list is full
 */
esp_err_t calibration_add_point(calibration_handle_t *handler, float code, float value);

/**
 * @brief Add multiple calibration points
 *
 * Either all points are added or none of them: the free space is checked
 * before the first point is stored.
 *
 * @param handler Pointer to calibration handle structure
 * @param points  Array of calibration points
 * @param count   Number of calibration points to add
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler or @p points is NULL, the handle
 *        is not initialized, or one of the points is not finite
 *      - `ESP_ERR_INVALID_STATE` if the points do not fit into the list
 */
esp_err_t calibration_add_points(calibration_handle_t *handler, const calibration_point_t *points, size_t count);

/**
 * @brief Get calibrated value by raw value
 *
 * The value is linearly interpolated between the two closest points. A raw
 * value equal to one of the points returns that point's calibrated value.
 * Raw values outside the range of the points are extrapolated using the
 * outermost segment, so the result is a linear continuation, not a clamp.
 *
 * @param handler    Pointer to calibration handle structure
 * @param code       Raw value, must be finite
 * @param[out] value Calculated calibrated value
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler or @p value is NULL, the handle
 *        is not initialized, or @p code is not finite
 *      - `ESP_ERR_INVALID_STATE` if less than two points were added
 */
esp_err_t calibration_get_value(const calibration_handle_t *handler, float code, float *value);

/**
 * @brief Free calibration handle
 *
 * Releases the point list. The handle can be initialized again after that.
 *
 * @param handler Pointer to calibration handle structure
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler is NULL
 *
 * @note The library is not thread-safe and does not lock the handle.
 * @note Fields of @ref calibration_handle_t are filled by the library and
 * must not be modified by the application.
 */
esp_err_t calibration_free(calibration_handle_t *handler);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __CALIBRATION_H__ */

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
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Approximation methods
 *
 * The methods interpolate the calibrated value between the calibration
 * points. Values outside the range of the points are always extrapolated with
 * the outermost segment, see @ref calibration_prepare().
 */
typedef enum
{
    /**
     * Piecewise linear interpolation of the value against the raw value.
     * Usually enough for a dense calibration table.
     */
    CALIBRATION_LINEAR = 0,

    /**
     * Piecewise linear interpolation of the value against the natural
     * logarithm of the raw value. Use it when the dependence is
     * exponential, e.g. for thermistors or photo resistors. Requires
     * positive raw values.
     */
    CALIBRATION_LOG_LINEAR,

    /**
     * Piecewise linear interpolation against the natural logarithms of both
     * the raw value and the calibrated value, e.g. for flow meters and other
     * sensors with a power-law transfer function. Requires positive raw
     * values and positive calibrated values.
     */
    CALIBRATION_LOG_LOG,

    /**
     * Monotone cubic Hermite interpolation (Fritsch-Carlson). Smoother than
     * the piecewise linear one and never overshoots the neighbouring points,
     * unlike an ordinary cubic spline. Requires at least three points.
     */
    CALIBRATION_PCHIP,

    /**
     * Monotone cubic Hermite interpolation against the natural logarithm of
     * the raw value. Requires at least three points and positive raw values.
     */
    CALIBRATION_PCHIP_LOG,
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
 *
 * The fields marked as internal are filled by the library.
 */
typedef struct
{
    calibration_method_t type;   //!< Approximation method, filled by calibration_init()
    calibration_point_t *points; //!< Ordered list of calibration points, filled by the library
    size_t count;                //!< Maximum number of calibration points, filled by calibration_init()
    size_t filled;               //!< Current number of calibration points, filled by the library
    float *slopes;               //!< Internal: derivatives of the monotone interpolation
    bool prepared;               //!< Internal: set by calibration_prepare()
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
 * @param type    Approximation method
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler is NULL or @p count is less than 2
 *      - `ESP_ERR_NOT_SUPPORTED` if @p type is unknown
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
 * The log methods require positive raw values, and
 * @ref CALIBRATION_LOG_LOG additionally requires positive calibrated values,
 * so such a point is rejected before it reaches the list.
 *
 * Adding a point invalidates the result of @ref calibration_prepare().
 *
 * @param handler Pointer to calibration handle structure
 * @param code    Raw value
 * @param value   Calibrated value
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler is NULL, the handle is not
 *        initialized, one of the values is not finite, or the point does not
 *        suit the selected method (non-positive raw value for the log
 *        methods, non-positive calibrated value for @ref CALIBRATION_LOG_LOG)
 *      - `ESP_ERR_INVALID_STATE` if the point list is full
 */
esp_err_t calibration_add_point(calibration_handle_t *handler, float code, float value);

/**
 * @brief Add multiple calibration points
 *
 * Either all points are added or none of them: the free space and the values
 * are checked before the first point is stored.
 *
 * Adding points invalidates the result of @ref calibration_prepare().
 *
 * @param handler Pointer to calibration handle structure
 * @param points  Array of calibration points
 * @param count   Number of calibration points to add
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler or @p points is NULL, the handle
 *        is not initialized, or one of the points is not finite or does not
 *        suit the selected method
 *      - `ESP_ERR_INVALID_STATE` if the points do not fit into the list
 */
esp_err_t calibration_add_points(calibration_handle_t *handler, const calibration_point_t *points, size_t count);

/**
 * @brief Prepare the handle for the calculations
 *
 * Checks that the list contains enough points for the selected method
 * (two for the linear methods, three for the monotone interpolation) and
 * computes the derivatives used by @ref CALIBRATION_PCHIP and
 * @ref CALIBRATION_PCHIP_LOG. For the linear methods the call only validates
 * the list, so the application does not have to branch on the method.
 *
 * The call also warns once that values outside the range of the points are
 * extrapolated.
 *
 * @param handler Pointer to calibration handle structure
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler is NULL or the handle is not
 *        initialized
 *      - `ESP_ERR_INVALID_STATE` if the list does not contain enough points
 *      - `ESP_ERR_NO_MEM` if the derivatives could not be allocated
 */
esp_err_t calibration_prepare(calibration_handle_t *handler);

/**
 * @brief Get calibrated value by raw value
 *
 * The value is interpolated with the method selected in
 * @ref calibration_init(). A raw value equal to one of the points returns
 * that point's calibrated value. Raw values outside the range of the points
 * are extrapolated using the outermost segment, so the result is a
 * continuation of the curve, not a clamp. Every such call is logged at the
 * debug level.
 *
 * @ref calibration_prepare() must be called after the points are added.
 *
 * @param handler    Pointer to calibration handle structure
 * @param code       Raw value, must be finite and positive for the log methods
 * @param[out] value Calculated calibrated value
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p handler or @p value is NULL, the handle
 *        is not initialized, @p code is not finite or is not positive while
 *        a log method is selected
 *      - `ESP_ERR_INVALID_STATE` if @ref calibration_prepare() was not called
 *        after the last change of the point list
 */
esp_err_t calibration_get_value(const calibration_handle_t *handler, float code, float *value);

/**
 * @brief Free calibration handle
 *
 * Releases the point list and the internal buffers. The handle can be
 * initialized again after that.
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

/**
 * Grid point
 */
typedef struct
{
    float x;     //!< Raw value of the first argument
    float y;     //!< Raw value of the second argument
    float value; //!< Calibrated value
} calibration_grid_point_t;

/**
 * Calibration grid
 *
 * A two-dimensional calibration table for quantities which depend on two
 * arguments, e.g. a pressure sensor compensated by temperature.
 *
 * The fields marked as internal are filled by the library.
 */
typedef struct
{
    calibration_grid_point_t *points; //!< Unordered list of grid points, filled by the library
    size_t rows;                      //!< Number of distinct Y values, filled by calibration_grid_init()
    size_t cols;                      //!< Number of distinct X values, filled by calibration_grid_init()
    size_t capacity;                  //!< Maximum number of grid points, filled by calibration_grid_init()
    size_t filled;                    //!< Current number of grid points, filled by the library
    float *xs;                        //!< Internal: sorted distinct X values
    float *ys;                        //!< Internal: sorted distinct Y values
    float *values;                    //!< Internal: dense matrix, rows by cols
    bool prepared;                    //!< Internal: set by calibration_grid_prepare()
} calibration_grid_t;

/**
 * @brief Init calibration grid
 *
 * Allocates memory to store @p rows times @p cols points and fills the
 * handle. The handle must be zeroed before the first call, e.g.
 * `calibration_grid_t grid = { 0 };`.
 *
 * @param grid Pointer to calibration grid structure
 * @param rows Number of distinct values of the second argument, at least 2
 * @param cols Number of distinct values of the first argument, at least 2
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p grid is NULL, @p rows or @p cols is less
 *        than 2, or their product overflows
 *      - `ESP_ERR_INVALID_STATE` if the grid is already initialized
 *      - `ESP_ERR_NO_MEM` if the point list could not be allocated
 */
esp_err_t calibration_grid_init(calibration_grid_t *grid, size_t rows, size_t cols);

/**
 * @brief Add grid point
 *
 * If a point with the same pair of arguments already exists, its calibrated
 * value is replaced. All three values must be finite.
 *
 * Adding a point invalidates the result of @ref calibration_grid_prepare().
 *
 * @param grid  Pointer to calibration grid structure
 * @param x     Raw value of the first argument
 * @param y     Raw value of the second argument
 * @param value Calibrated value
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p grid is NULL, the grid is not
 *        initialized, or one of the values is not finite
 *      - `ESP_ERR_INVALID_STATE` if the grid is full
 */
esp_err_t calibration_grid_add_point(calibration_grid_t *grid, float x, float y, float value);

/**
 * @brief Add multiple grid points
 *
 * Either all points are added or none of them.
 *
 * @param grid   Pointer to calibration grid structure
 * @param points Array of grid points
 * @param count  Number of grid points to add
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p grid or @p points is NULL, the grid is
 *        not initialized, or one of the points is not finite
 *      - `ESP_ERR_INVALID_STATE` if the points do not fit into the grid
 */
esp_err_t calibration_grid_add_points(calibration_grid_t *grid, const calibration_grid_point_t *points, size_t count);

/**
 * @brief Prepare the grid for the calculations
 *
 * Checks that every cell of the grid is filled and that the points contain
 * exactly @p rows distinct values of the second argument and @p cols distinct
 * values of the first one, then builds the sorted axes and the dense matrix.
 *
 * The call also warns once that values outside the grid are extrapolated.
 *
 * @param grid Pointer to calibration grid structure
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p grid is NULL or the grid is not initialized
 *      - `ESP_ERR_INVALID_STATE` if the grid is incomplete or the number of
 *        distinct values does not match the declared size
 *      - `ESP_ERR_NO_MEM` if the internal buffers could not be allocated
 */
esp_err_t calibration_grid_prepare(calibration_grid_t *grid);

/**
 * @brief Get calibrated value by the pair of raw values
 *
 * The value is bilinearly interpolated between the four surrounding grid
 * points. Values outside the grid are extrapolated using the outermost cells,
 * so the result is a continuation of the surface, not a clamp. Every such
 * call is logged at the debug level.
 *
 * @ref calibration_grid_prepare() must be called after the points are added.
 *
 * @param grid       Pointer to calibration grid structure
 * @param x          Raw value of the first argument, must be finite
 * @param y          Raw value of the second argument, must be finite
 * @param[out] value Calculated calibrated value
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p grid or @p value is NULL, the grid is not
 *        initialized, or @p x or @p y is not finite
 *      - `ESP_ERR_INVALID_STATE` if @ref calibration_grid_prepare() was not
 *        called after the last change of the point list
 */
esp_err_t calibration_grid_get_value(const calibration_grid_t *grid, float x, float y, float *value);

/**
 * @brief Free calibration grid
 *
 * Releases the point list and the internal buffers. The grid can be
 * initialized again after that.
 *
 * @param grid Pointer to calibration grid structure
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p grid is NULL
 *
 * @note The library is not thread-safe and does not lock the grid.
 * @note Fields of @ref calibration_grid_t are filled by the library and must
 * not be modified by the application.
 */
esp_err_t calibration_grid_free(calibration_grid_t *grid);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __CALIBRATION_H__ */

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
 * @file calibration.c
 *
 * ESP-IDF Multi-point calibration (approximation) library
 *
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "calibration_private.h"
#include <math.h>
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "calibration";

bool calibration_log_code(calibration_method_t type)
{
    return type == CALIBRATION_LOG_LINEAR || type == CALIBRATION_LOG_LOG
           || type == CALIBRATION_PCHIP_LOG;
}

bool calibration_log_value(calibration_method_t type)
{
    return type == CALIBRATION_LOG_LOG;
}

static bool method_is_supported(calibration_method_t type)
{
    switch (type)
    {
        case CALIBRATION_LINEAR:
        case CALIBRATION_LOG_LINEAR:
        case CALIBRATION_LOG_LOG:
        case CALIBRATION_PCHIP:
        case CALIBRATION_PCHIP_LOG:
            return true;
        default:
            return false;
    }
}

static bool point_is_valid(const calibration_handle_t *handler, float code, float value)
{
    if (!isfinite(code) || !isfinite(value))
        return false;
    if (calibration_log_code(handler->type) && code <= 0)
        return false;
    if (calibration_log_value(handler->type) && value <= 0)
        return false;
    return true;
}

static float calc_linear_value(float x1, float y1, float x2, float y2, float x)
{
    float kx = (y2 - y1) / (x2 - x1);
    return y1 + kx * (x - x1);
}

/**
 * Find the segment the raw value belongs to
 */
static size_t find_segment(const calibration_handle_t *handler, float code)
{
    size_t pos;

    for (pos = 0; pos < handler->filled; pos++)
        if (handler->points[pos].code > code)
            break;

    if (pos > 0)
        pos--;
    if (pos >= handler->filled - 1)
        pos = handler->filled - 2;

    return pos;
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t calibration_init(calibration_handle_t *handler, size_t count, calibration_method_t type)
{
    CHECK_ARG(handler && count > 1);

    if (!method_is_supported(type))
    {
        ESP_LOGE(TAG, "Unknown approximation method %d", (int)type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (handler->points)
    {
        ESP_LOGE(TAG, "Handler already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    handler->type = type;
    handler->count = count;
    handler->filled = 0;
    handler->slopes = NULL;
    handler->prepared = false;
    handler->points = calloc(handler->count, sizeof(calibration_point_t));
    if (!handler->points)
    {
        ESP_LOGE(TAG, "Could not allocate memory for calibration points");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t calibration_add_point(calibration_handle_t *handler, float code, float value)
{
    CHECK_ARG(handler && handler->points);

    if (!point_is_valid(handler, code, value))
    {
        ESP_LOGE(TAG, "Calibration point does not suit the approximation method: code %f, value %f",
                 code, value);
        return ESP_ERR_INVALID_ARG;
    }

    // FIXME: inefficient
    for (size_t i = 0; i < handler->filled; i++)
        if (handler->points[i].code == code)
        {
            handler->points[i].value = value;
            handler->prepared = false;
            return ESP_OK;
        }

    if (handler->filled == handler->count)
    {
        ESP_LOGE(TAG, "List of calibration points is full");
        return ESP_ERR_INVALID_STATE;
    }

    size_t pos;
    for (pos = 0; pos < handler->filled; pos++)
        if (handler->points[pos].code > code)
            break;

    if (pos < handler->filled)
        memmove(handler->points + pos + 1, handler->points + pos, sizeof(calibration_point_t) * (handler->filled - pos));

    handler->points[pos].code = code;
    handler->points[pos].value = value;

    handler->filled++;
    handler->prepared = false;

    return ESP_OK;
}

esp_err_t calibration_add_points(calibration_handle_t *handler, const calibration_point_t *points, size_t count)
{
    CHECK_ARG(handler && handler->points && points);

    if (count > handler->count - handler->filled)
    {
        ESP_LOGE(TAG, "Cannot add %u point(s), %u of %u slots are used",
                 (unsigned)count, (unsigned)handler->filled, (unsigned)handler->count);
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < count; i++)
        if (!point_is_valid(handler, points[i].code, points[i].value))
        {
            ESP_LOGE(TAG, "Calibration point %u does not suit the approximation method: code %f, value %f",
                     (unsigned)i, points[i].code, points[i].value);
            return ESP_ERR_INVALID_ARG;
        }

    for (size_t i = 0; i < count; i++)
        CHECK(calibration_add_point(handler, points[i].code, points[i].value));

    return ESP_OK;
}

esp_err_t calibration_prepare(calibration_handle_t *handler)
{
    CHECK_ARG(handler && handler->points);

    const size_t required = handler->type == CALIBRATION_PCHIP || handler->type == CALIBRATION_PCHIP_LOG ? 3 : 2;

    if (handler->filled < required)
    {
        ESP_LOGE(TAG, "Not enough calibration points for method %d, need at least %u, have %u",
                 (int)handler->type, (unsigned)required, (unsigned)handler->filled);
        return ESP_ERR_INVALID_STATE;
    }

    if (handler->type == CALIBRATION_PCHIP || handler->type == CALIBRATION_PCHIP_LOG)
        CHECK(calibration_pchip_prepare(handler));

    handler->prepared = true;

    ESP_LOGW(TAG, "Values outside the point range are extrapolated with the outermost segment");

    return ESP_OK;
}

esp_err_t calibration_get_value(const calibration_handle_t *handler, float code, float *value)
{
    CHECK_ARG(handler && handler->points && value);

    if (!isfinite(code))
    {
        ESP_LOGE(TAG, "Raw value must be finite: %f", code);
        return ESP_ERR_INVALID_ARG;
    }

    if (calibration_log_code(handler->type) && code <= 0)
    {
        ESP_LOGE(TAG, "Raw value must be positive for the logarithmic methods: %f", code);
        return ESP_ERR_INVALID_ARG;
    }

    if (!handler->prepared)
    {
        ESP_LOGE(TAG, "The handle is not prepared, call calibration_prepare() after adding the points");
        return ESP_ERR_INVALID_STATE;
    }

    if (code < handler->points[0].code || code > handler->points[handler->filled - 1].code)
        ESP_LOGD(TAG, "Raw value %f is outside [%f, %f], extrapolating", code,
                 handler->points[0].code, handler->points[handler->filled - 1].code);

    if (handler->type == CALIBRATION_PCHIP || handler->type == CALIBRATION_PCHIP_LOG)
    {
        *value = calibration_pchip_get_value(handler, code);
        return ESP_OK;
    }

    const size_t pos = find_segment(handler, code);
    const calibration_point_t *p1 = handler->points + pos;
    const calibration_point_t *p2 = p1 + 1;

    switch (handler->type)
    {
        case CALIBRATION_LOG_LINEAR:
            *value = calc_linear_value(logf(p1->code), p1->value, logf(p2->code), p2->value, logf(code));
            break;
        case CALIBRATION_LOG_LOG:
            *value = expf(calc_linear_value(logf(p1->code), logf(p1->value),
                                            logf(p2->code), logf(p2->value), logf(code)));
            break;
        default:
            *value = calc_linear_value(p1->code, p1->value, p2->code, p2->value, code);
            break;
    }

    return ESP_OK;
}

esp_err_t calibration_free(calibration_handle_t *handler)
{
    CHECK_ARG(handler);

    if (handler->points)
    {
        free(handler->points);
        handler->points = NULL;
    }

    if (handler->slopes)
    {
        free(handler->slopes);
        handler->slopes = NULL;
    }

    handler->filled = 0;
    handler->prepared = false;

    return ESP_OK;
}

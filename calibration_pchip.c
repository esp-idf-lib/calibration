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
 * @file calibration_pchip.c
 *
 * Monotone cubic Hermite interpolation (Fritsch-Carlson) for the calibration
 * library.
 */
#include "calibration_private.h"
#include <math.h>
#include <esp_log.h>
#include <stdlib.h>

static const char *TAG = "calibration_pchip";

/**
 * Raw value in the interpolation domain
 */
static float axis(const calibration_handle_t *handler, float code)
{
    return calibration_log_code(handler->type) ? logf(code) : code;
}

/**
 * One-sided derivative at the end of the point list
 */
static float end_slope(float h0, float h1, float d0, float d1)
{
    float m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);

    if (m * d0 <= 0)
        return 0;
    if (d0 * d1 <= 0 && fabsf(m) > 3 * fabsf(d0))
        return 3 * d0;
    return m;
}

esp_err_t calibration_pchip_prepare(calibration_handle_t *handler)
{
    const size_t n = handler->filled;

    if (n < 3)
        return ESP_ERR_INVALID_STATE;

    if (!handler->slopes)
    {
        handler->slopes = calloc(handler->count, sizeof(float));
        if (!handler->slopes)
        {
            ESP_LOGE(TAG, "Could not allocate memory for the interpolation slopes");
            return ESP_ERR_NO_MEM;
        }
    }

    const calibration_point_t *points = handler->points;
    float *slopes = handler->slopes;

    // The method reduces to the linear one when the calibrated values are
    // constant, all slopes are zero in that case
    bool constant = true;
    for (size_t i = 1; i < n; i++)
        if (points[i].value != points[0].value)
        {
            constant = false;
            break;
        }
    if (constant)
    {
        for (size_t i = 0; i < n; i++)
            slopes[i] = 0;
        return ESP_OK;
    }

    for (size_t i = 1; i + 1 < n; i++)
    {
        const float h0 = axis(handler, points[i].code) - axis(handler, points[i - 1].code);
        const float h1 = axis(handler, points[i + 1].code) - axis(handler, points[i].code);
        const float d0 = (points[i].value - points[i - 1].value) / h0;
        const float d1 = (points[i + 1].value - points[i].value) / h1;

        if (d0 * d1 <= 0)
            slopes[i] = 0;
        else
        {
            const float w0 = 2 * h1 + h0;
            const float w1 = h1 + 2 * h0;
            slopes[i] = (w0 + w1) / (w0 / d0 + w1 / d1);
        }
    }

    const float h_first = axis(handler, points[1].code) - axis(handler, points[0].code);
    const float h_second = axis(handler, points[2].code) - axis(handler, points[1].code);
    slopes[0] = end_slope(h_first, h_second,
                          (points[1].value - points[0].value) / h_first,
                          (points[2].value - points[1].value) / h_second);

    const float h_last = axis(handler, points[n - 1].code) - axis(handler, points[n - 2].code);
    const float h_prev = axis(handler, points[n - 2].code) - axis(handler, points[n - 3].code);
    slopes[n - 1] = end_slope(h_last, h_prev,
                              (points[n - 1].value - points[n - 2].value) / h_last,
                              (points[n - 2].value - points[n - 3].value) / h_prev);

    return ESP_OK;
}

float calibration_pchip_get_value(const calibration_handle_t *handler, float code)
{
    const size_t n = handler->filled;
    const calibration_point_t *points = handler->points;
    const float *slopes = handler->slopes;

    const float x = axis(handler, code);
    const float x_first = axis(handler, points[0].code);
    const float x_last = axis(handler, points[n - 1].code);

    // Outside the point range the curve is continued linearly: the cubic
    // segment would grow too fast far from the calibration table
    if (x <= x_first)
        return points[0].value + slopes[0] * (x - x_first);
    if (x >= x_last)
        return points[n - 1].value + slopes[n - 1] * (x - x_last);

    size_t pos;
    for (pos = 0; pos + 2 < n; pos++)
        if (axis(handler, points[pos + 1].code) > x)
            break;

    const float x0 = axis(handler, points[pos].code);
    const float x1 = axis(handler, points[pos + 1].code);
    const float h = x1 - x0;
    const float t = (x - x0) / h;

    const float t2 = t * t;
    const float t3 = t2 * t;

    return (2 * t3 - 3 * t2 + 1) * points[pos].value
           + (t3 - 2 * t2 + t) * h * slopes[pos]
           + (-2 * t3 + 3 * t2) * points[pos + 1].value
           + (t3 - t2) * h * slopes[pos + 1];
}

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
 * @file calibration_grid.c
 *
 * Bilinear interpolation over a two-dimensional calibration table.
 */
#include "calibration.h"
#include <math.h>
#include <esp_log.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "calibration_grid";

static int cmp_float(const void *a, const void *b)
{
    const float fa = *(const float *)a;
    const float fb = *(const float *)b;

    return fa < fb ? -1 : (fa > fb ? 1 : 0);
}

static bool has_point(const calibration_grid_t *grid, float x, float y)
{
    for (size_t i = 0; i < grid->filled; i++)
        if (grid->points[i].x == x && grid->points[i].y == y)
            return true;
    return false;
}

/**
 * Find the cell the value belongs to
 */
static size_t find_cell(const float *axis, size_t count, float value, bool *outside)
{
    if (value < axis[0])
    {
        *outside = true;
        return 0;
    }
    if (value > axis[count - 1])
    {
        *outside = true;
        return count - 2;
    }

    size_t lo = 0;
    size_t hi = count - 1;
    while (hi - lo > 1)
    {
        const size_t mid = lo + (hi - lo) / 2;
        if (axis[mid] <= value)
            lo = mid;
        else
            hi = mid;
    }
    return lo;
}

static size_t index_of(const float *axis, size_t count, float value)
{
    for (size_t i = 0; i < count; i++)
        if (axis[i] == value)
            return i;
    return count;
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t calibration_grid_init(calibration_grid_t *grid, size_t rows, size_t cols)
{
    CHECK_ARG(grid && rows > 1 && cols > 1);

    if (rows > SIZE_MAX / cols)
    {
        ESP_LOGE(TAG, "Grid size %u x %u overflows", (unsigned)rows, (unsigned)cols);
        return ESP_ERR_INVALID_ARG;
    }

    if (grid->points)
    {
        ESP_LOGE(TAG, "Grid already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    grid->rows = rows;
    grid->cols = cols;
    grid->capacity = rows * cols;
    grid->filled = 0;
    grid->xs = NULL;
    grid->ys = NULL;
    grid->values = NULL;
    grid->prepared = false;
    grid->points = calloc(grid->capacity, sizeof(calibration_grid_point_t));
    if (!grid->points)
    {
        ESP_LOGE(TAG, "Could not allocate memory for the grid points");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t calibration_grid_add_point(calibration_grid_t *grid, float x, float y, float value)
{
    CHECK_ARG(grid && grid->points);

    if (!isfinite(x) || !isfinite(y) || !isfinite(value))
    {
        ESP_LOGE(TAG, "Grid point must be finite: x %f, y %f, value %f", x, y, value);
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < grid->filled; i++)
        if (grid->points[i].x == x && grid->points[i].y == y)
        {
            grid->points[i].value = value;
            grid->prepared = false;
            return ESP_OK;
        }

    if (grid->filled == grid->capacity)
    {
        ESP_LOGE(TAG, "Grid is full");
        return ESP_ERR_INVALID_STATE;
    }

    grid->points[grid->filled].x = x;
    grid->points[grid->filled].y = y;
    grid->points[grid->filled].value = value;
    grid->filled++;
    grid->prepared = false;

    return ESP_OK;
}

esp_err_t calibration_grid_add_points(calibration_grid_t *grid, const calibration_grid_point_t *points, size_t count)
{
    CHECK_ARG(grid && grid->points && points);

    for (size_t i = 0; i < count; i++)
        if (!isfinite(points[i].x) || !isfinite(points[i].y) || !isfinite(points[i].value))
        {
            ESP_LOGE(TAG, "Grid point %u must be finite: x %f, y %f, value %f", (unsigned)i,
                     points[i].x, points[i].y, points[i].value);
            return ESP_ERR_INVALID_ARG;
        }

    // the batch may replace existing points, so only the new ones are counted
    size_t needed = 0;
    for (size_t i = 0; i < count; i++)
    {
        bool known = has_point(grid, points[i].x, points[i].y);
        for (size_t j = 0; !known && j < i; j++)
            known = points[j].x == points[i].x && points[j].y == points[i].y;
        if (!known)
            needed++;
    }

    if (needed > grid->capacity - grid->filled)
    {
        ESP_LOGE(TAG, "Cannot add %u point(s), %u of %u cells are used",
                 (unsigned)needed, (unsigned)grid->filled, (unsigned)grid->capacity);
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < count; i++)
        CHECK(calibration_grid_add_point(grid, points[i].x, points[i].y, points[i].value));

    return ESP_OK;
}

esp_err_t calibration_grid_prepare(calibration_grid_t *grid)
{
    CHECK_ARG(grid && grid->points);

    if (grid->filled != grid->capacity)
    {
        ESP_LOGE(TAG, "Grid is incomplete: %u of %u cells are filled",
                 (unsigned)grid->filled, (unsigned)grid->capacity);
        return ESP_ERR_INVALID_STATE;
    }

    free(grid->xs);
    free(grid->ys);
    free(grid->values);
    grid->xs = NULL;
    grid->ys = NULL;
    grid->values = NULL;
    grid->prepared = false;

    float *xs = malloc(grid->capacity * sizeof(float));
    float *ys = malloc(grid->capacity * sizeof(float));
    if (!xs || !ys)
    {
        ESP_LOGE(TAG, "Could not allocate memory for the grid axes");
        free(xs);
        free(ys);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < grid->filled; i++)
    {
        xs[i] = grid->points[i].x;
        ys[i] = grid->points[i].y;
    }

    qsort(xs, grid->filled, sizeof(float), cmp_float);
    qsort(ys, grid->filled, sizeof(float), cmp_float);

    size_t cols = 1;
    for (size_t i = 1; i < grid->filled; i++)
        if (xs[i] != xs[cols - 1])
            xs[cols++] = xs[i];

    size_t rows = 1;
    for (size_t i = 1; i < grid->filled; i++)
        if (ys[i] != ys[rows - 1])
            ys[rows++] = ys[i];

    if (cols != grid->cols || rows != grid->rows)
    {
        ESP_LOGE(TAG, "Grid contains %u distinct X and %u distinct Y values, expected %u and %u",
                 (unsigned)cols, (unsigned)rows, (unsigned)grid->cols, (unsigned)grid->rows);
        free(xs);
        free(ys);
        return ESP_ERR_INVALID_STATE;
    }

    float *values = calloc(grid->capacity, sizeof(float));
    if (!values)
    {
        ESP_LOGE(TAG, "Could not allocate memory for the grid values");
        free(xs);
        free(ys);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < grid->filled; i++)
    {
        const size_t col = index_of(xs, cols, grid->points[i].x);
        const size_t row = index_of(ys, rows, grid->points[i].y);
        if (col == cols || row == rows)
        {
            ESP_LOGE(TAG, "Grid point %u is out of the axes", (unsigned)i);
            free(xs);
            free(ys);
            free(values);
            return ESP_ERR_INVALID_STATE;
        }
        values[row * cols + col] = grid->points[i].value;
    }

    grid->xs = xs;
    grid->ys = ys;
    grid->values = values;
    grid->prepared = true;

    ESP_LOGW(TAG, "Values outside the grid are extrapolated with the outermost cells");

    return ESP_OK;
}

esp_err_t calibration_grid_get_value(const calibration_grid_t *grid, float x, float y, float *value)
{
    CHECK_ARG(grid && grid->values && value);

    if (!isfinite(x) || !isfinite(y))
    {
        ESP_LOGE(TAG, "Grid arguments must be finite: x %f, y %f", x, y);
        return ESP_ERR_INVALID_ARG;
    }

    if (!grid->prepared)
    {
        ESP_LOGE(TAG, "The grid is not prepared, call calibration_grid_prepare() after adding the points");
        return ESP_ERR_INVALID_STATE;
    }

    bool outside = false;
    const size_t col = find_cell(grid->xs, grid->cols, x, &outside);
    const size_t row = find_cell(grid->ys, grid->rows, y, &outside);

    if (outside)
        ESP_LOGD(TAG, "Arguments %f, %f are outside the grid, extrapolating", x, y);

    const float x0 = grid->xs[col];
    const float x1 = grid->xs[col + 1];
    const float y0 = grid->ys[row];
    const float y1 = grid->ys[row + 1];

    const float dx = (x - x0) / (x1 - x0);
    const float dy = (y - y0) / (y1 - y0);

    const float v00 = grid->values[row * grid->cols + col];
    const float v01 = grid->values[row * grid->cols + col + 1];
    const float v10 = grid->values[(row + 1) * grid->cols + col];
    const float v11 = grid->values[(row + 1) * grid->cols + col + 1];

    *value = (1 - dx) * (1 - dy) * v00 + dx * (1 - dy) * v01
             + (1 - dx) * dy * v10 + dx * dy * v11;

    return ESP_OK;
}

esp_err_t calibration_grid_free(calibration_grid_t *grid)
{
    CHECK_ARG(grid);

    free(grid->points);
    free(grid->xs);
    free(grid->ys);
    free(grid->values);

    grid->points = NULL;
    grid->xs = NULL;
    grid->ys = NULL;
    grid->values = NULL;
    grid->filled = 0;
    grid->prepared = false;

    return ESP_OK;
}

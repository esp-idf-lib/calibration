/*
 * This example code is in the Public Domain.
 */

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <calibration.h>

static const char *TAG = "calibration_grid_example";

// Pressure sensor calibrated against the temperature: three temperatures,
// four raw values each
#define ROWS 3
#define COLS 4

static const calibration_grid_point_t points[] =
{
    // 0 deg C
    { .x = 100,  .y = 0, .value = 1001 },
    { .x = 200,  .y = 0, .value = 2003 },
    { .x = 300,  .y = 0, .value = 2996 },
    { .x = 400,  .y = 0, .value = 4009 },
    // 25 deg C
    { .x = 100,  .y = 25, .value = 1003 },
    { .x = 200,  .y = 25, .value = 2001 },
    { .x = 300,  .y = 25, .value = 3002 },
    { .x = 400,  .y = 25, .value = 3998 },
    // 50 deg C
    { .x = 100,  .y = 50, .value = 1007 },
    { .x = 200,  .y = 50, .value = 2008 },
    { .x = 300,  .y = 50, .value = 3009 },
    { .x = 400,  .y = 50, .value = 4004 },
};

static const calibration_grid_point_t arguments[] =
{
    { .x = 150, .y = 12 },
    { .x = 250, .y = 12 },
    { .x = 350, .y = 37 },
    { .x = 300, .y = 25 },
    { .x = 450, .y = 60 },
};

void app_main()
{
    const size_t num_points = sizeof(points) / sizeof(points[0]);
    const size_t num_arguments = sizeof(arguments) / sizeof(arguments[0]);

    calibration_grid_t grid = { 0 };

    ESP_LOGI(TAG, "Grid %u x %u, %u points", ROWS, COLS, (unsigned)num_points);

    ESP_ERROR_CHECK(calibration_grid_init(&grid, ROWS, COLS));
    ESP_ERROR_CHECK(calibration_grid_add_points(&grid, points, num_points));
    ESP_ERROR_CHECK(calibration_grid_prepare(&grid));

    printf("\n%10s%10s%14s\n", "raw", "temp", "compensated");
    for (size_t i = 0; i < num_arguments; i++)
    {
        float v;
        ESP_ERROR_CHECK(calibration_grid_get_value(&grid, arguments[i].x, arguments[i].y, &v));
        printf("%10.2f%10.2f%14.3f\n", arguments[i].x, arguments[i].y, v);
    }
    printf("\n");

    ESP_ERROR_CHECK(calibration_grid_free(&grid));
}

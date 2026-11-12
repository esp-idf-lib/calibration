/*
 * This example code is in the Public Domain.
 */

#include <stdio.h>
#include <esp_log.h>
#include <calibration.h>

static const char *TAG = "calibration_example";

static const calibration_point_t points[] =
{
    { .code = 20,  .value = 40 },
    { .code = 10,  .value = 25 },
    { .code = 15,  .value = 31 },
    { .code = 30,  .value = 55 },
    { .code = 100, .value = 250 },
    { .code = 50,  .value = 80 },
};

static const float test_points[] =
{
    0, -40, 15, 16, 30, 300, 31, 35, 40
};

void app_main()
{
    const size_t num_points = sizeof(points) / sizeof(points[0]);
    const size_t num_test_points = sizeof(test_points) / sizeof(test_points[0]);

    calibration_handle_t c = { 0 };
    ESP_ERROR_CHECK(calibration_init(&c, num_points, CALIBRATION_LINEAR));
    ESP_ERROR_CHECK(calibration_add_points(&c, points, num_points));

    ESP_LOGI(TAG, "Calibration points:");
    for (size_t i = 0; i < c.filled; i++)
        ESP_LOGI(TAG, "Point %zu: %.2f ~ %.2f", i, c.points[i].code, c.points[i].value);

    ESP_LOGI(TAG, "Test points:");
    for (size_t i = 0; i < num_test_points; i++)
    {
        float v;
        ESP_ERROR_CHECK(calibration_get_value(&c, test_points[i], &v));
        ESP_LOGI(TAG, "Test %zu: %.2f ~ %.2f", i, test_points[i], v);
    }

    ESP_ERROR_CHECK(calibration_free(&c));
}

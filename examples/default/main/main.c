/*
 * This example code is in the Public Domain.
 */

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <calibration.h>

static const char *TAG = "calibration_example";

// Raw values are positive, so every method can be applied to this table
static const calibration_point_t points[] =
{
    { .code = 10,  .value = 25 },
    { .code = 15,  .value = 31 },
    { .code = 20,  .value = 40 },
    { .code = 30,  .value = 55 },
    { .code = 50,  .value = 80 },
    { .code = 100, .value = 250 },
};

static const calibration_method_t methods[] =
{
    CALIBRATION_LINEAR,
    CALIBRATION_LOG_LINEAR,
    CALIBRATION_LOG_LOG,
    CALIBRATION_PCHIP,
    CALIBRATION_PCHIP_LOG,
};

static const char *method_name(calibration_method_t method)
{
    switch (method)
    {
        case CALIBRATION_LINEAR:
            return "linear";
        case CALIBRATION_LOG_LINEAR:
            return "log-linear";
        case CALIBRATION_LOG_LOG:
            return "log-log";
        case CALIBRATION_PCHIP:
            return "pchip";
        case CALIBRATION_PCHIP_LOG:
            return "pchip-log";
        default:
            return "unknown";
    }
}

static const float test_points[] =
{
    5, 12, 17, 25, 40, 70, 120
};

void app_main()
{
    const size_t num_points = sizeof(points) / sizeof(points[0]);
    const size_t num_methods = sizeof(methods) / sizeof(methods[0]);
    const size_t num_test_points = sizeof(test_points) / sizeof(test_points[0]);

    calibration_handle_t handlers[sizeof(methods) / sizeof(methods[0])];

    memset(handlers, 0, sizeof(handlers));

    ESP_LOGI(TAG, "Calibration points:");
    for (size_t i = 0; i < num_points; i++)
        ESP_LOGI(TAG, "Point %zu: %.2f ~ %.2f", i, points[i].code, points[i].value);

    for (size_t i = 0; i < num_methods; i++)
    {
        ESP_ERROR_CHECK(calibration_init(&handlers[i], num_points, methods[i]));
        ESP_ERROR_CHECK(calibration_add_points(&handlers[i], points, num_points));
        ESP_ERROR_CHECK(calibration_prepare(&handlers[i]));
    }

    ESP_LOGI(TAG, "Interpolated values:");
    printf("\n%10s", "raw");
    for (size_t i = 0; i < num_methods; i++)
        printf("%12s", method_name(methods[i]));
    printf("\n");

    for (size_t i = 0; i < num_test_points; i++)
    {
        printf("%10.2f", test_points[i]);
        for (size_t j = 0; j < num_methods; j++)
        {
            float v;
            ESP_ERROR_CHECK(calibration_get_value(&handlers[j], test_points[i], &v));
            printf("%12.3f", v);
        }
        printf("\n");
    }
    printf("\n");

    for (size_t i = 0; i < num_methods; i++)
        ESP_ERROR_CHECK(calibration_free(&handlers[i]));
}

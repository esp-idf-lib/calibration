# esp-idf-lib/calibration

[![Build examples](https://github.com/esp-idf-lib/calibration/actions/workflows//build.yml/badge.svg)](https://github.com/esp-idf-lib/calibration/actions/workflows//build.yml)
[![Build docs](https://github.com/esp-idf-lib/calibration/actions/workflows//build-docs.yml/badge.svg)](https://github.com/esp-idf-lib/calibration/actions/workflows//build-docs.yml)
[![Validation](https://github.com/esp-idf-lib/calibration/actions/workflows//validate-component.yml/badge.svg)](https://github.com/esp-idf-lib/calibration/actions/workflows//validate-component.yml)

Multi-point calibration (approximation) library.

* [Documentation](https://esp-idf-lib.github.io/calibration/)
* [Repository](https://github.com/esp-idf-lib/calibration)
* [Issues](https://github.com/esp-idf-lib/calibration/issues)
* [Discussions and questions](https://esp-idf-lib/core/discussions)
* [Component page at the ESP Component Registry](https://components.espressif.com/components/esp-idf-lib/calibration)

A sensor rarely reports the quantity it measures. A thermistor returns a resistance, an ADC channel returns a code, a bridge returns a voltage, and the relation between that number and the physical value is non-linear, differs from sensor to sensor and drifts with the batch. Datasheets usually give a formula, but the formula describes a typical part, not the one on the board.

This component implements the other approach: a table of calibration points. You write down a few pairs "raw code -> true value", measured once with a reference instrument, and the library computes the value for any code in between. The table is small - a handful of points for a thermistor, a couple of dozens for an RTD, a few hundred for a pressure sensor - and the evaluation is a lookup plus a couple of multiplications: no series expansion, no logarithms unless the method asks for them, no floating-point heavy code on a microcontroller.

The points are stored sorted by the raw code, so they may be added in any order, a repeated code replaces the earlier point, and a batch may be added at once. Nothing is computed by `calibration_add_point()`: `calibration_prepare()` validates the table and precomputes what the chosen method needs. A bad table is therefore reported once, and not on every reading.

## Interpolation methods

The shape of the curve between the points matters. The method is selected when the handle is initialized and can not be changed afterwards.

| Method | Points | Curve between the points | Typical use |
| --- | --- | --- | --- |
| `CALIBRATION_LINEAR` | 2 | polyline: continuous, derivative jumps at the points | a nearly linear sensor; a dense table, where the segments are short |
| `CALIBRATION_LOG_LINEAR` | 2 | straight line in `(ln(code), value)` | an exponential sensor: NTC thermistor, photo resistor |
| `CALIBRATION_LOG_LOG` | 2 | straight line in `(ln(code), ln(value))` | a power-law sensor: flow meter, hot-wire anemometer |
| `CALIBRATION_PCHIP` | 3 | monotone cubic: smooth, never leaves the interval between the neighbouring points | a smooth sensor with a sparse table: RTD, thermocouple, load cell |
| `CALIBRATION_PCHIP_LOG` | 3 | monotone cubic over `ln(code)` | an exponential sensor with a sparse table |

`CALIBRATION_LINEAR` is the simplest and the cheapest: the value never leaves the interval between the two surrounding points, but the derivative jumps at every point. It is the right choice for a sensor which is nearly linear over the range of interest, and for a table dense enough for the kinks to be invisible.

`CALIBRATION_LOG_LINEAR` and `CALIBRATION_LOG_LOG` interpolate against the natural logarithm of the raw code (and, for the second one, of the calibrated value). An NTC thermistor, a photo resistor or any other sensor whose resistance depends exponentially on the measured quantity need exactly that: over a wide range a linear table is useless, while five points against a logarithm are accurate to a fraction of a degree. The log methods require positive raw codes; `CALIBRATION_LOG_LOG` requires positive calibrated values as well.

`CALIBRATION_PCHIP` uses monotone cubic Hermite interpolation (the Fritsch-Carlson variant): a smooth curve with a continuous derivative, which passes exactly through the points and, unlike an ordinary cubic spline, never overshoots the interval between two neighbouring values. No overshoot matters when the value is physically bounded or the sensor is known to be monotone: a spline may report a negative pressure or a temperature above the boiling point between two perfectly sane points. It needs at least three points and is the best default for a smooth sensor with a sparse table.

The difference is not academic: on a thermistor table of five points the linear method may be off by several degrees in the middle of the range, the log-linear one by a fraction of a degree, and the monotone cubic one by less than the accuracy of the sensor itself.

## Two-dimensional table

A quantity may depend on two arguments: the output of a pressure sensor depends on the temperature of the die, the reading of a gas sensor depends on the humidity, the voltage of a battery depends on the load. A list of points can not describe such a surface, so the component has a separate type, `calibration_grid_t`: a rectangular table of `rows` by `cols` cells with bilinear interpolation inside a cell.

The table must be filled completely, and the number of distinct values along each axis must match the declared size. `calibration_grid_prepare()` refuses an incomplete table, because a hole in the table would otherwise be interpolated across silently.

## Values outside the range

Both tables extrapolate: outside the range the outermost segment (or the outermost row and column of cells) is continued. This is deliberate - a sensor in an unplanned mode still has to return something plausible, and refusing the value would break the application exactly when the reading matters most - but it is not silent: `calibration_prepare()` warns once that the table does not cover the range, and every value taken from outside the range is logged at the debug level.

## Example

An NTC thermistor calibrated with five points; the raw codes are ADC readings, the values are degrees Celsius measured with a reference thermometer.

```c
#include <esp_err.h>
#include <esp_log.h>
#include <calibration.h>

static calibration_handle_t thermistor;

void app_main()
{
    ESP_ERROR_CHECK(calibration_init(&thermistor, 5, CALIBRATION_LOG_LINEAR));

    ESP_ERROR_CHECK(calibration_add_point(&thermistor, 8060.0f, -20.0f));
    ESP_ERROR_CHECK(calibration_add_point(&thermistor, 3720.0f, 0.0f));
    ESP_ERROR_CHECK(calibration_add_point(&thermistor, 1810.0f, 25.0f));
    ESP_ERROR_CHECK(calibration_add_point(&thermistor, 925.0f, 50.0f));
    ESP_ERROR_CHECK(calibration_add_point(&thermistor, 505.0f, 75.0f));

    ESP_ERROR_CHECK(calibration_prepare(&thermistor));

    float temperature;
    esp_err_t res = calibration_get_value(&thermistor, 1810.0f, &temperature);
    if (res == ESP_OK)
        ESP_LOGI("app", "Temperature: %.2f C", temperature);
    else
        ESP_LOGE("app", "Could not calibrate the reading: %s", esp_err_to_name(res));
}
```

The handle is safe to query from any task once it is prepared, and it is released with `calibration_free()`. See `examples/default` for the one-dimensional methods and `examples/grid` for the two-dimensional table.

## Installation

```sh
idf.py add-dependency esp-idf-lib/calibration
```

## Support

For questions and discussions about the component, please use
[Discussions](https://github.com/esp-idf-lib/core/discussions)
at [esp-idf-lib/core](https://github.com/esp-idf-lib/core).

## Contributing

Please read [CONTRIBUTING.md](https://github.com/esp-idf-lib/core/blob/main/CONTRIBUTING.md)
at [esp-idf-lib/core](https://github.com/esp-idf-lib/core).

calibration: Multi-point calibration library
============================================

A sensor rarely reports the quantity it measures. A thermistor returns a
resistance, an ADC channel returns a code, a bridge returns a voltage, and the
relation between that number and the physical value is non-linear, differs
from sensor to sensor and drifts with the batch. Datasheets usually give a
formula, but the formula describes a typical part, not the one on the board.

The component solves that with a table of calibration points: you write down a
few pairs "raw code -> true value", measured once with a reference instrument,
and the library computes the value for any code in between. The table is small
(a handful of points for a thermistor, a couple of dozens for an RTD and a few
hundred for a pressure sensor) and it is cheap in both code size and run time:
a lookup and a couple of multiplications, no floating-point series, no
logarithms unless the method asks for them.

Points are stored sorted by the raw code, so they may be added in any order, a
repeated code replaces the earlier point, and a whole batch may be added at
once. Nothing is computed in `calibration_add_point()`: `calibration_prepare()`
validates the table and precomputes what the chosen method needs, which keeps
the search and the evaluation branch-free and lets the library report a bad
table once, instead of failing on every reading.

Interpolation methods
---------------------

The shape of the curve between the points matters, and different sensors need
different shapes. The method is chosen when the handle is initialized and can
not be changed afterwards.

``CALIBRATION_LINEAR``
    A polyline: the value is interpolated linearly between the two surrounding
    points. The result is continuous, never leaves the interval between the
    neighbouring values and never needs more than two points, but its
    derivative jumps at every point. This is the right choice for a sensor
    which is nearly linear over the range of interest, and for a dense table
    where the segments are short enough for the kinks to be invisible.

``CALIBRATION_LOG_LINEAR``
    A straight line in the ``(ln(code), value)`` plane: the raw code is
    logarithmic, the calibrated value is linear. An NTC thermistor, a photo
    resistor or any other sensor whose resistance depends exponentially on the
    measured quantity need exactly that: a linear table over such a sensor is
    useless over a wide range, a logarithmic one is accurate with five points.
    The raw codes must be positive.

``CALIBRATION_LOG_LOG``
    A straight line in the ``(ln(code), ln(value))`` plane, for a sensor with a
    power-law response, ``value = a * code^b``: a flow meter, a hot-wire
    anemometer, a strain gauge bridge with a non-linear excitation. Both the
    raw codes and the calibrated values must be positive.

``CALIBRATION_PCHIP``
    Monotone cubic Hermite interpolation (the Fritsch-Carlson variant): a
    smooth curve with a continuous derivative, which passes exactly through the
    points and, unlike an ordinary cubic spline, never overshoots the interval
    between two neighbouring values. No overshoot matters when the value is
    physically bounded or when the sensor is known to be monotone: a spline may
    report a negative pressure or a temperature above the boiling point between
    two perfectly sane points. It needs at least three points and is the best
    default for a smooth sensor with a sparse table: an RTD, a thermocouple, a
    load cell.

``CALIBRATION_PCHIP_LOG``
    The same monotone cubic curve over the logarithm of the raw code, for an
    exponential sensor whose table is too sparse even for the log-linear
    method.

No method fits every sensor, and the difference is not academic: on a
thermistor table of five points the linear method may be off by several
degrees in the middle of the range, the log-linear one by a fraction of a
degree, and the monotone cubic one by less than the sensor itself.

Two-dimensional table
---------------------

A quantity may depend on two arguments: the output of a pressure sensor
depends on the temperature of the die as well, the reading of a gas sensor
depends on the humidity, the voltage of a battery depends on the load. A list
of points can not describe such a surface, so the component has a separate
type, `calibration_grid_t`, with a rectangular table of ``rows`` by ``cols``
cells and bilinear interpolation inside a cell. The table must be filled
completely, and the number of distinct values along each axis must match the
declared size, otherwise `calibration_grid_prepare()` refuses it: a hole in
the table would otherwise be interpolated across silently.

Values outside the range
------------------------

Both the one-dimensional and the two-dimensional tables extrapolate: outside
the range the outermost segment (or the outermost row and column of cells) is
continued. It is a deliberate choice - a sensor in an unplanned mode still has
to return something plausible, and refusing the value would break the
application exactly when the reading matters most - but it is not silent:
`calibration_prepare()` warns once that the table does not cover the range,
and every value taken from outside the range is logged at the debug level.

Example
-------

An NTC thermistor calibrated with five points; the raw codes are ADC
readings, the values are degrees Celsius measured with a reference
thermometer.

.. code-block:: c

    #include <esp_err.h>
    #include <esp_log.h>
    #include <calibration.h>

    static calibration_handle_t thermistor;

    void app_main(void)
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

The same handle is safe to query from any task once it is prepared, and it is
released with `calibration_free()`. See `examples/default` for the
one-dimensional methods and `examples/grid` for the two-dimensional table.

 .. doxygenfile:: calibration.h

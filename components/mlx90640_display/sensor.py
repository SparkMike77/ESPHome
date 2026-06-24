"""
Sensor sub-platform for mlx90640_display.

Allows optionally surfacing min/max detected temperatures as
Home Assistant sensor entities:

  sensor:
    - platform: mlx90640_display
      mlx90640_id: thermal_cam
      min_temperature:
        name: "Thermal Min Temp"
      max_temperature:
        name: "Thermal Max Temp"
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import MLX90640Component, mlx90640_ns

CONF_MLX90640_ID = "mlx90640_id"
CONF_MIN_TEMPERATURE = "min_temperature"
CONF_MAX_TEMPERATURE = "max_temperature"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MLX90640_ID): cv.use_id(MLX90640Component),
        cv.Optional(CONF_MIN_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MAX_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MLX90640_ID])

    if CONF_MIN_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_MIN_TEMPERATURE])
        cg.add(parent.set_min_temp_sensor(sens))

    if CONF_MAX_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_MAX_TEMPERATURE])
        cg.add(parent.set_max_temp_sensor(sens))

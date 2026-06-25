"""ESPHome external component: MLX90640 thermal camera reader."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

CODEOWNERS = ["@local"]
DEPENDENCIES = []
AUTO_LOAD = ["sensor"]

mlx90640_ns = cg.esphome_ns.namespace("mlx90640")
MLX90640Component = mlx90640_ns.class_(
    "MLX90640Component", cg.PollingComponent
)

CONF_MINTEMP = "mintemp"
CONF_MAXTEMP = "maxtemp"
CONF_REFRESH_RATE = "refresh_rate"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MLX90640Component),
        cv.Optional(CONF_MINTEMP, default=15.0): cv.float_,
        cv.Optional(CONF_MAXTEMP, default=40.0): cv.float_,
        # Sensor refresh rate register value (mlx90640_refreshrate_t):
        #   0x00=0.5Hz 0x01=1Hz 0x02=2Hz 0x03=4Hz 0x04=8Hz
        #   0x05=16Hz  0x06=32Hz  0x07=64Hz
        cv.Optional(CONF_REFRESH_RATE, default=0x04): cv.hex_uint8_t,
        cv.Optional(CONF_UPDATE_INTERVAL, default="200ms"): cv.update_interval,
    }
).extend(cv.polling_component_schema("200ms"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_mintemp(config[CONF_MINTEMP]))
    cg.add(var.set_maxtemp(config[CONF_MAXTEMP]))
    cg.add(var.set_refresh_rate(config[CONF_REFRESH_RATE]))

    # No external library dependencies — all driver code is bundled in the
    # component directory and compiled as component source, so Wire.h is
    # always visible (unlike PlatformIO's .piolibdeps compilation context).

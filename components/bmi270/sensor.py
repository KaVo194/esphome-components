import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor

DEPENDENCIES = ["i2c"]

bmi270_ns = cg.esphome_ns.namespace("bmi270")
BMI270Component = bmi270_ns.class_("BMI270Component", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(BMI270Component),
        cv.Optional("odr_hz", default=100): cv.int_range(min=12, max=1600),
        cv.Optional("accel_x"): sensor.sensor_schema(unit_of_measurement="m/s²"),
        cv.Optional("accel_y"): sensor.sensor_schema(unit_of_measurement="m/s²"),
        cv.Optional("accel_z"): sensor.sensor_schema(unit_of_measurement="m/s²"),
        cv.Optional("gyro_x"):  sensor.sensor_schema(unit_of_measurement="°/s"),
        cv.Optional("gyro_y"):  sensor.sensor_schema(unit_of_measurement="°/s"),
        cv.Optional("gyro_z"):  sensor.sensor_schema(unit_of_measurement="°/s"),
    })
    .extend(i2c.i2c_device_schema(0x68))
    .extend(cv.polling_component_schema("100ms"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[cg.CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_odr(config["odr_hz"]))

    for k in ("accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z"):
        if k in config:
            s = await sensor.new_sensor(config[k])
            cg.add(getattr(var, k).set_parent(s))

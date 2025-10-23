from esphome.components import stepper, modbus
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ACCELERATION,
    CONF_DECELERATION,
    CONF_ID,
    CONF_MAX_SPEED,
    CONF_POSITION,
    CONF_TARGET,
    CONF_SPEED,
)
from esphome import automation

AUTO_LOAD = ["modbus"]

servo42d_rs485_ns = cg.esphome_ns.namespace("servo42d_rs485")
Servo42dRs485 = servo42d_rs485_ns.class_("Servo42dRs485", stepper.Stepper, modbus.ModbusDevice, cg.PollingComponent)

# Actions
EnableMotorAction = servo42d_rs485_ns.class_("EnableMotorAction", automation.Action)
EmergencyStopAction = servo42d_rs485_ns.class_("EmergencyStopAction", automation.Action)
ReleaseProtectionAction = servo42d_rs485_ns.class_("ReleaseProtectionAction", automation.Action)
CalibrateMotorAction = servo42d_rs485_ns.class_("CalibrateMotorAction", automation.Action)
GoToZeroAction = servo42d_rs485_ns.class_("GoToZeroAction", automation.Action)
SetWorkModeAction = servo42d_rs485_ns.class_("SetWorkModeAction", automation.Action)
SetWorkingCurrentAction = servo42d_rs485_ns.class_("SetWorkingCurrentAction", automation.Action)

def validate_steps_per_revolution(value):
    value = cv.string(value)
    for suffix in (
        "steps", "pulses"
    ):
        if value.endswith(suffix):
            value = value[: -len(suffix)]

    try:
        value = float(value)
    except ValueError:
        # pylint: disable=raise-missing-from
        raise cv.Invalid(f"Expected steps per revolution as floating point number, got {value}")

    if value <= 0:
        raise cv.Invalid("Steps per revolution must be larger than 0 steps!")

    return value

def validate_microsteps(value):
    value = cv.string(value)
    for suffix in (
        "steps", "pulses"
    ):
        if value.endswith(suffix):
            value = value[: -len(suffix)]

    try:
        value = int(value)
    except ValueError:
        # pylint: disable=raise-missing-from
        raise cv.Invalid(f"Expected microsteps as integer, got {value}")

    if value <= 0:
        raise cv.Invalid("Microsteps must be larger than 0 steps!")

    if value > 256:
        raise cv.Invalid("Microsteps must be smaller or equal to 256 steps!")

    return value

def validate_speed(value):
    value = cv.string(value)
    valuetype = "steps/s"
    for suffix in ("steps/s", "step/s", "pulses/s", "pluse/s"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/s"
    for suffix in ("steps/min", "step/min", "pulses/min", "pluse/min"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/min"
    for suffix in ("RPM", "rpm", "revolutions/min"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/min"
    for suffix in ("RPS", "rps", "revolutions/s"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/s"

    if value == "inf":
        return 1e6

    try:
        value = float(value)
    except ValueError:
        # pylint: disable=raise-missing-from
        raise cv.Invalid(f"Expected speed as floating point number, got {value}")

    if value <= 0:
        raise cv.Invalid("Speed must be larger than 0 steps/s!")
    if valuetype == "steps/min":
        value = value/60.0
        valuetype = "steps/s"
    if valuetype == "revolutions/min":
        value = value/60.0
        valuetype = "revolutions/s"

    return {"type": valuetype, "value": value}

def validate_acceleration(value):
    value = cv.string(value)
    valuetype = "steps/s^2"
    for suffix in ("steps/s^2", "steps/s*s", "steps/s/s", "steps/ss", "steps/(s*s)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/s^2"
    for suffix in ("steps/min^2", "steps/min*min", "steps/min/min", "steps/minmin", "steps/(min*min)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/min^2"
    for suffix in ("revolutions/s^2", "revolutions/s*s", "revolutions/s/s", "revolutions/ss", "revolutions/(s*s)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/s^2"
    for suffix in ("revolutions/min^2", "revolutions/min*min", "revolutions/min/min", "revolutions/minmin", "revolutions/(min*min)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/min^2"

    if value == "inf":
        return 1e6

    try:
        value = float(value)
    except ValueError:
        # pylint: disable=raise-missing-from
        raise cv.Invalid(f"Expected acceleration as floating point number, got {value}")

    if value <= 0:
        raise cv.Invalid("Acceleration must be larger than 0 steps/s^2!")
    if valuetype == "steps/min^2":
        value = value/3600.0
        valuetype = "steps/s^2"
    if valuetype == "revolutions/min^2":
        value = value/3600.0
        valuetype = "revolutions/s^2"

    return {"type": valuetype, "value": value}

CONF_STEPS_PER_REVOLUTION = "steps_per_revolution"
CONF_MICROSTEPS = "microsteps"
CONF_SLEEP_WHEN_DONE ="sleep_when_done"

CONFIG_SCHEMA = stepper.STEPPER_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(Servo42dRs485),
        cv.Optional(CONF_STEPS_PER_REVOLUTION, default=3200.0): validate_steps_per_revolution,
        cv.Optional(CONF_MICROSTEPS, default=16): validate_microsteps,
        cv.Required(CONF_MAX_SPEED): validate_speed,
        cv.Optional(CONF_ACCELERATION, default="inf"): validate_acceleration,
        cv.Optional(CONF_DECELERATION, default="inf"): validate_acceleration,
        cv.Optional(CONF_SLEEP_WHEN_DONE, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA).extend(modbus.modbus_device_schema(0x01)).extend(cv.polling_component_schema("100ms"))

async def preprocess_speed(config):
    max_speed_config = config[CONF_MAX_SPEED]
    if max_speed_config["type"] == "revolutions/s":
        steps_per_revolution = config[CONF_STEPS_PER_REVOLUTION]
        max_speed_revolutions_per_second = max_speed_config["value"]
        max_speed_steps_per_second = (max_speed_revolutions_per_second * steps_per_revolution)
        config[CONF_MAX_SPEED] = max_speed_steps_per_second
    elif max_speed_config["type"] == "steps/s":
        config[CONF_MAX_SPEED] = max_speed_config["value"]
    else:
        raise cv.Invalid("Invalid speed unit specified for max_speed.")
    acceleration_config = config[CONF_ACCELERATION]
    if acceleration_config["type"] == "revolutions/s^2":
        steps_per_revolution = config[CONF_STEPS_PER_REVOLUTION]
        acceleration_revolutions_per_second = acceleration_config["value"]
        acceleration_steps_per_second = (acceleration_revolutions_per_second * steps_per_revolution)
        config[CONF_ACCELERATION] = acceleration_steps_per_second
    elif acceleration_config["type"] == "steps/s^2":
        config[CONF_ACCELERATION] = acceleration_config["value"]
    else:
        raise cv.Invalid("Invalid acceleration unit specified for acceleration.")
    deceleration_config = config[CONF_DECELERATION]
    if deceleration_config["type"] == "revolutions/s^2":
        steps_per_revolution = config[CONF_STEPS_PER_REVOLUTION]
        deceleration_revolutions_per_second = deceleration_config["value"]
        deceleration_steps_per_second = (deceleration_revolutions_per_second * steps_per_revolution)
        config[CONF_DECELERATION] = deceleration_steps_per_second
    elif deceleration_config["type"] == "steps/s^2":
        config[CONF_DECELERATION] = deceleration_config["value"]
    else:
        raise cv.Invalid("Invalid deceleration unit specified for deceleration.")
    return config

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_microsteps(config[CONF_MICROSTEPS]))
    cg.add(var.set_steps_per_revolution(config[CONF_STEPS_PER_REVOLUTION]))
    cg.add(var.set_sleep_when_done(config[CONF_SLEEP_WHEN_DONE]))
    preprocessed_config = await preprocess_speed(config)
    await cg.register_component(var, config)
    await stepper.register_stepper(var, preprocessed_config)
    await modbus.register_modbus_device(var, config)


# ============================================================================
# Actions
# ============================================================================

CONF_ENABLE = "enable"
CONF_DIRECTION = "direction"
CONF_MODE = "mode"
CONF_CURRENT = "current"

@automation.register_action(
    "servo42d_rs485.enable_motor",
    EnableMotorAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_ENABLE): cv.templatable(cv.boolean),
    }),
)
async def servo42d_enable_motor_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_ENABLE], args, bool)
    cg.add(var.set_enable(template_))
    return var

@automation.register_action(
    "servo42d_rs485.emergency_stop",
    EmergencyStopAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
    }),
)
async def servo42d_emergency_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "servo42d_rs485.release_protection",
    ReleaseProtectionAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
    }),
)
async def servo42d_release_protection_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "servo42d_rs485.calibrate",
    CalibrateMotorAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
    }),
)
async def servo42d_calibrate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "servo42d_rs485.go_to_zero",
    GoToZeroAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_ENABLE): cv.templatable(cv.boolean),
        cv.Optional(CONF_SPEED, default=500): cv.templatable(cv.uint16_t),
        cv.Optional(CONF_DIRECTION, default=0): cv.templatable(cv.uint16_t),
    }),
)
async def servo42d_go_to_zero_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_ENABLE], args, bool)
    cg.add(var.set_enable(template_))
    
    template_ = await cg.templatable(config[CONF_SPEED], args, cg.uint16)
    cg.add(var.set_speed(template_))
    
    template_ = await cg.templatable(config[CONF_DIRECTION], args, cg.uint16)
    cg.add(var.set_direction(template_))
    return var

@automation.register_action(
    "servo42d_rs485.set_work_mode",
    SetWorkModeAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_MODE): cv.templatable(cv.uint16_t),
    }),
)
async def servo42d_set_work_mode_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_MODE], args, cg.uint16)
    cg.add(var.set_mode(template_))
    return var

@automation.register_action(
    "servo42d_rs485.set_working_current",
    SetWorkingCurrentAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_CURRENT): cv.templatable(cv.uint16_t),
    }),
)
async def servo42d_set_working_current_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_CURRENT], args, cg.uint16)
    cg.add(var.set_current(template_))
    return var


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
DisableMotorAction = servo42d_rs485_ns.class_("DisableMotorAction", automation.Action)
EmergencyStopAction = servo42d_rs485_ns.class_("EmergencyStopAction", automation.Action)
RunContinuousAction = servo42d_rs485_ns.class_("RunContinuousAction", automation.Action)
StopMotorAction = servo42d_rs485_ns.class_("StopMotorAction", automation.Action)
HomeAction = servo42d_rs485_ns.class_("HomeAction", automation.Action)
ResetPositionAction = servo42d_rs485_ns.class_("ResetPositionAction", automation.Action)
CalibrateMotorAction = servo42d_rs485_ns.class_("CalibrateMotorAction", automation.Action)
ReleaseProtectionAction = servo42d_rs485_ns.class_("ReleaseProtectionAction", automation.Action)
RestartMotorAction = servo42d_rs485_ns.class_("RestartMotorAction", automation.Action)
SetWorkModeAction = servo42d_rs485_ns.class_("SetWorkModeAction", automation.Action)
SetWorkingCurrentAction = servo42d_rs485_ns.class_("SetWorkingCurrentAction", automation.Action)
SetHoldingCurrentPercentAction = servo42d_rs485_ns.class_("SetHoldingCurrentPercentAction", automation.Action)
SetMicrosteppingAction = servo42d_rs485_ns.class_("SetMicrosteppingAction", automation.Action)
KeyLockAction = servo42d_rs485_ns.class_("KeyLockAction", automation.Action)
KeyUnlockAction = servo42d_rs485_ns.class_("KeyUnlockAction", automation.Action)
SetTargetAction = servo42d_rs485_ns.class_("SetTargetAction", automation.Action)
ReportPositionAction = servo42d_rs485_ns.class_("ReportPositionAction", automation.Action)

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
        raise cv.Invalid(
            "steps_per_revolution must be > 0! NOTE: The encoder is used internally. You must specify exactly how many steps (microsteps) correspond to one mechanical revolution. Wrong values will result in incorrect position and speed calculation!"
        )
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
    """Validate speed with flexible units.
    
    Supported units:
    - steps/s, step/s (always allowed)
    - steps/min, step/min (always allowed)
    - RPM, rpm, revolutions/min (requires steps_per_revolution)
    - RPS, rps, revolutions/s (requires steps_per_revolution)
    - degrees/s, deg/s, °/s (requires steps_per_revolution)
    - degrees/min, deg/min, °/min (requires steps_per_revolution)
    """
    value = cv.string(value)
    valuetype = "steps/s"
    
    # Steps-based units (always allowed - microsteps)
    for suffix in ("steps/s", "step/s", "pulses/s", "pulse/s"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/s"
    for suffix in ("steps/min", "step/min", "pulses/min", "pulse/min"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/min"
    
    # Revolution-based units (requires steps_per_revolution)
    for suffix in ("RPM", "rpm", "revolutions/min", "revolution/min"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/min"
    for suffix in ("RPS", "rps", "revolutions/s", "revolution/s"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/s"
    
    # Degree-based units (requires steps_per_revolution)
    for suffix in ("degrees/s", "degree/s", "deg/s", "°/s"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "degrees/s"
    for suffix in ("degrees/min", "degree/min", "deg/min", "°/min"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "degrees/min"

    if value == "inf":
        return {"type": "steps/s", "value": 1e6}

    try:
        value = float(value)
    except ValueError:
        # pylint: disable=raise-missing-from
        raise cv.Invalid(f"Expected speed as floating point number, got {value}")

    if value <= 0:
        raise cv.Invalid("Speed must be larger than 0!")
    
    # Normalize time units to per-second
    if valuetype == "steps/min":
        value = value/60.0
        valuetype = "steps/s"
    if valuetype == "revolutions/min":
        value = value/60.0
        valuetype = "revolutions/s"
    if valuetype == "degrees/min":
        value = value/60.0
        valuetype = "degrees/s"

    return {"type": valuetype, "value": value}

def validate_acceleration(value):
    """Validate acceleration with flexible units.
    
    Supported units:
    - steps/s^2 (always allowed - microsteps)
    - steps/min^2 (always allowed - microsteps)
    - revolutions/s^2 (requires steps_per_revolution)
    - revolutions/min^2 (requires steps_per_revolution)
    - degrees/s^2 (requires steps_per_revolution)
    - degrees/min^2 (requires steps_per_revolution)
    """
    value = cv.string(value)
    valuetype = "steps/s^2"
    
    # Steps-based units (always allowed - microsteps)
    for suffix in ("steps/s^2", "steps/s*s", "steps/s/s", "steps/ss", "steps/(s*s)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/s^2"
    for suffix in ("steps/min^2", "steps/min*min", "steps/min/min", "steps/minmin", "steps/(min*min)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "steps/min^2"
    
    # Revolution-based units (requires steps_per_revolution)
    for suffix in ("revolutions/s^2", "revolutions/s*s", "revolutions/s/s", "revolutions/ss", "revolutions/(s*s)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/s^2"
    for suffix in ("revolutions/min^2", "revolutions/min*min", "revolutions/min/min", "revolutions/minmin", "revolutions/(min*min)"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "revolutions/min^2"
    
    # Degree-based units (requires steps_per_revolution)
    for suffix in ("degrees/s^2", "degrees/s*s", "degrees/s/s", "degrees/ss", "degrees/(s*s)", "deg/s^2", "°/s^2"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "degrees/s^2"
    for suffix in ("degrees/min^2", "degrees/min*min", "degrees/min/min", "degrees/minmin", "degrees/(min*min)", "deg/min^2", "°/min^2"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            valuetype = "degrees/min^2"

    if value == "inf":
        return {"type": "steps/s^2", "value": 1e6}

    try:
        value = float(value)
    except ValueError:
        # pylint: disable=raise-missing-from
        raise cv.Invalid(f"Expected acceleration as floating point number, got {value}")

    if value <= 0:
        raise cv.Invalid("Acceleration must be larger than 0!")
    
    # Normalize time units to per-second squared
    if valuetype == "steps/min^2":
        value = value/3600.0
        valuetype = "steps/s^2"
    if valuetype == "revolutions/min^2":
        value = value/3600.0
        valuetype = "revolutions/s^2"
    if valuetype == "degrees/min^2":
        value = value/3600.0
        valuetype = "degrees/s^2"

    return {"type": valuetype, "value": value}

CONF_STEPS_PER_REVOLUTION = "steps_per_revolution"
CONF_MICROSTEPS = "microsteps"
CONF_SLEEP_WHEN_DONE = "sleep_when_done"
CONF_CONTROL_MODE = "control_mode"
CONF_WORKING_CURRENT = "working_current"
CONF_HOLDING_CURRENT_PERCENT = "holding_current_percent"
CONF_HOME_AT_STARTUP = "home_at_startup"
CONF_USE_VIRTUAL_HOME = "use_virtual_home"
CONF_VIRTUAL_HOME_ANGLE = "virtual_home_angle"
CONF_HOMING_SPEED = "homing_speed"
CONF_HOMING_DIRECTION = "homing_direction"
CONF_HOMING_CURRENT = "homing_current"
CONF_AUTO_SCREEN_OFF = "auto_screen_off"
CONF_LOCK_KEYS_AT_STARTUP = "lock_keys_at_startup"
CONF_EN_PIN_ACTIVE = "en_pin_active"
CONF_POST_ARRIVAL_HOLD_MS = "post_arrival_hold_ms"

# Motor type enum (affects default/max working current)
CONF_SERVO_TYPE = "servo_type"
MotorType = servo42d_rs485_ns.enum("MotorType")
SERVO_TYPES = {
    "SERVO28D": 28,
    "SERVO35D": 35,
    "SERVO42D": 42,
    "SERVO57D": 57,
}

# Work mode enum
WorkMode = servo42d_rs485_ns.enum("WorkMode")
WORK_MODES = {
    "CR_OPEN": 0,    # Pulse control, open-loop, 400 RPM max
    "CR_CLOSE": 1,   # Pulse control, closed-loop, 1500 RPM max  
    "CR_VFOC": 2,    # Pulse control, FOC, 3000 RPM max
    "SR_OPEN": 3,    # Serial/Modbus, open-loop
    "SR_CLOSE": 4,   # Serial/Modbus, closed-loop
    "SR_VFOC": 5,    # Serial/Modbus, FOC (RECOMMENDED for RS485 control!)
}

# Homing direction enum  
HomingDirection = servo42d_rs485_ns.enum("HomingDirection")
HOMING_DIRECTIONS = {
    "CW": 0,         # Clockwise (only for real homing with endstop)
    "CCW": 1,        # Counter-clockwise (only for real homing with endstop)
    "NEAREST": 2,    # Shortest path (only for virtual homing without endstop)
}

# EN pin active mode enum
EnPinActive = servo42d_rs485_ns.enum("EnPinActive")
EN_PIN_ACTIVE_MODES = {
    "LOW": 0,        # Active low (L)
    "HIGH": 1,       # Active high (H)
    "ALWAYS": 2,     # Active always (Hold)
}

def _apply_type_current_defaults(cfg):
    """Apply type-specific default and max validation for working_current.
    - working_current: amperes (float) after cv.current
    - servo_type: enum numeric (28/35/42/57)
    """
    motor_type = cfg.get(CONF_SERVO_TYPE, SERVO_TYPES["SERVO42D"])  # 28/35/42/57
    if motor_type == SERVO_TYPES["SERVO57D"]:
        default_a, max_a = 3.2, 5.2
        hm_default_a = 0.4
    elif motor_type == SERVO_TYPES["SERVO42D"]:
        default_a, max_a = 1.6, 3.0
        hm_default_a = 0.8
    elif motor_type == SERVO_TYPES["SERVO28D"]:
        default_a, max_a = 0.6, 3.0
        hm_default_a = 0.2
    else:  # SERVO35D
        default_a, max_a = 0.8, 3.0
        hm_default_a = 0.2

    if CONF_WORKING_CURRENT in cfg and cfg[CONF_WORKING_CURRENT] is not None and cfg[CONF_WORKING_CURRENT] != "auto":
        wc = float(cfg[CONF_WORKING_CURRENT])
        if wc < 0 or wc > max_a:
            raise cv.Invalid(f"working_current {wc}A exceeds max {max_a}A for servo_type")
    else:
        cfg[CONF_WORKING_CURRENT] = default_a

    # homing_current: default and max identical policy; applies when use_virtual_home=true
    if CONF_HOMING_CURRENT in cfg and cfg[CONF_HOMING_CURRENT] is not None and cfg[CONF_HOMING_CURRENT] != "auto":
        hc = float(cfg[CONF_HOMING_CURRENT])
        if hc < 0 or hc > max_a:
            raise cv.Invalid(f"homing_current {hc}A exceeds max {max_a}A for servo_type")
    else:
        cfg[CONF_HOMING_CURRENT] = hm_default_a
    return cfg

CONFIG_SCHEMA = cv.All(
    stepper.STEPPER_SCHEMA.extend(
        {
            cv.Required(CONF_ID): cv.declare_id(Servo42dRs485),
            cv.Required(CONF_STEPS_PER_REVOLUTION): validate_steps_per_revolution,  # Jetzt verpflichtend!
            cv.Optional(CONF_MICROSTEPS, default=16): validate_microsteps,
            cv.Required(CONF_MAX_SPEED): validate_speed,
            cv.Optional(CONF_ACCELERATION, default="inf"): validate_acceleration,
            cv.Optional(CONF_DECELERATION, default="inf"): validate_acceleration,
            cv.Optional(CONF_SLEEP_WHEN_DONE, default=False): cv.boolean,
            # Motor configuration
            cv.Optional(CONF_SERVO_TYPE, default="SERVO42D"): cv.enum(SERVO_TYPES, upper=True),
            cv.Optional(CONF_CONTROL_MODE, default="SR_VFOC"): cv.enum(WORK_MODES, upper=True),
            # Working current in amperes; type-specific default/max applied via mapping-level validator below
            cv.Optional(CONF_WORKING_CURRENT): cv.All(
                cv.current,
                cv.float_range(min=0, max=5.2),  # global physical max bound
            ),  # e.g. 1.5A or 1500mA
            # Homing current (used only for noLimit/virtual homing). Same validation as working_current.
            cv.Optional(CONF_HOMING_CURRENT): cv.All(
                cv.current,
                cv.float_range(min=0, max=5.2),
            ),
            cv.Optional(CONF_HOLDING_CURRENT_PERCENT, default="50%"): cv.All(cv.percentage, cv.float_range(min=0.1, max=0.9)),  # e.g. 50% or 0.5
            cv.Optional(CONF_EN_PIN_ACTIVE, default="ALWAYS"): cv.enum(EN_PIN_ACTIVE_MODES, upper=True),  # Default: always active
            # Homing configuration
            cv.Optional(CONF_HOME_AT_STARTUP, default=False): cv.boolean,
            cv.Optional(CONF_USE_VIRTUAL_HOME, default=False): cv.boolean,
            cv.Optional(CONF_VIRTUAL_HOME_ANGLE): cv.All(cv.angle,  cv.float_range(min=0, max=360)),  # e.g. 90, 90deg, 90°
            cv.Optional(CONF_HOMING_SPEED): validate_speed,  # Flexible units, no default
            cv.Optional(CONF_HOMING_DIRECTION, default="CW"): cv.enum(HOMING_DIRECTIONS, upper=True),
            # Display configuration
            cv.Optional(CONF_AUTO_SCREEN_OFF, default=True): cv.boolean,  # Auto turn off screen after 15s
            cv.Optional(CONF_LOCK_KEYS_AT_STARTUP, default=False): cv.boolean,
            # Optional hold time before auto-disable (supports ms/s/min like "500ms", "1s")
            cv.Optional(CONF_POST_ARRIVAL_HOLD_MS, default="0ms"): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA).extend(modbus.modbus_device_schema(0x01)),
    _apply_type_current_defaults,
)

def _validate_virtual_homing_direction(cfg):
    """Only allow direction=NEAREST when using virtual homing (software/0_Mode).
    Current config uses use_virtual_home as the switch for virtual behavior.
    """
    try:
        dir_val = cfg.get(CONF_HOMING_DIRECTION)
        use_virtual = cfg.get(CONF_USE_VIRTUAL_HOME, False)
        if dir_val == HOMING_DIRECTIONS.get("NEAREST") and not use_virtual:
            raise cv.Invalid("homing.direction 'NEAREST' ist nur mit virtuellem Homing (use_virtual_home: true) erlaubt.")
    except Exception:
        # Be conservative: don't block other validations
        pass
    return cfg

# Chain additional mapping-level validations
CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _validate_virtual_homing_direction)

def convert_to_steps(value_config, steps_per_revolution, param_name):
    """Convert speed/acceleration value to microsteps-based units.
    steps_per_revolution is always set (mandatory)."""
    value_type = value_config["type"]
    value = value_config["value"]
    # Steps-based units - already microsteps, no conversion needed
    if value_type in ("steps/s", "steps/s^2"):
        return value
    # Revolution-based units - convert to microsteps
    if value_type in ("revolutions/s", "revolutions/s^2"):
        return value * steps_per_revolution
    # Degree-based units - convert to microsteps
    if value_type in ("degrees/s", "degrees/s^2"):
        return value * (steps_per_revolution / 360.0)
    raise cv.Invalid(f"Invalid unit type '{value_type}' for '{param_name}'.")

async def preprocess_speed(config):
    """Convert all speed/acceleration values to microsteps-based units. steps_per_revolution is always present (mandatory)."""
    steps_per_revolution = config[CONF_STEPS_PER_REVOLUTION]
    # Convert max_speed
    max_speed_config = config[CONF_MAX_SPEED]
    config[CONF_MAX_SPEED] = convert_to_steps(max_speed_config, steps_per_revolution, "max_speed")
    # Convert acceleration
    acceleration_config = config[CONF_ACCELERATION]
    config[CONF_ACCELERATION] = convert_to_steps(acceleration_config, steps_per_revolution, "acceleration")
    # Convert deceleration
    deceleration_config = config[CONF_DECELERATION]
    config[CONF_DECELERATION] = convert_to_steps(deceleration_config, steps_per_revolution, "deceleration")
    # Convert homing_speed if set
    if CONF_HOMING_SPEED in config:
        homing_speed_config = config[CONF_HOMING_SPEED]
        config[CONF_HOMING_SPEED] = convert_to_steps(homing_speed_config, steps_per_revolution, "homing_speed")
    return config

async def to_code(config):
    # Preprocess speed/acceleration values first
    preprocessed_config = await preprocess_speed(config)
    
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_microsteps(config[CONF_MICROSTEPS]))
    
    # Set steps_per_revolution (has default of 200)
    if CONF_STEPS_PER_REVOLUTION in config:
        cg.add(var.set_steps_per_revolution(config[CONF_STEPS_PER_REVOLUTION]))
    
    cg.add(var.set_sleep_when_done(config[CONF_SLEEP_WHEN_DONE]))
    
    # Motor configuration (use original config for non-speed values)
    if CONF_CONTROL_MODE in config:
        cg.add(var.set_control_mode(config[CONF_CONTROL_MODE]))
    # working_current: already defaulted and validated by CONFIG_SCHEMA; convert A -> mA
    if CONF_WORKING_CURRENT in config:
        cg.add(var.set_working_current(int(config[CONF_WORKING_CURRENT] * 1000)))
    # homing_current: convert A -> mA
    if CONF_HOMING_CURRENT in config:
        cg.add(var.set_homing_current(int(config[CONF_HOMING_CURRENT] * 1000)))
    if CONF_HOLDING_CURRENT_PERCENT in config:
        # Convert from ratio to percentage (cv.percentage returns float in [-1..1])
        cg.add(var.set_holding_current_percent(int(config[CONF_HOLDING_CURRENT_PERCENT] * 100)))
    if CONF_EN_PIN_ACTIVE in config:
        cg.add(var.set_en_pin_active(config[CONF_EN_PIN_ACTIVE]))
    
    # Homing configuration
    cg.add(var.set_home_at_startup(config[CONF_HOME_AT_STARTUP]))
    cg.add(var.set_use_virtual_home(config[CONF_USE_VIRTUAL_HOME]))
    if CONF_VIRTUAL_HOME_ANGLE in config:
        # Convert from float degrees to int, ensure 0-359 range
        cg.add(var.set_virtual_home_angle(int(config[CONF_VIRTUAL_HOME_ANGLE]) % 360))
    if CONF_HOMING_SPEED in preprocessed_config:
        # Use preprocessed value (already converted to steps/s)
        cg.add(var.set_homing_speed(preprocessed_config[CONF_HOMING_SPEED]))
    cg.add(var.set_homing_direction(config[CONF_HOMING_DIRECTION]))
    
    # Display configuration
    cg.add(var.set_auto_screen_off(config[CONF_AUTO_SCREEN_OFF]))
    cg.add(var.set_lock_keys_at_startup(config[CONF_LOCK_KEYS_AT_STARTUP]))
    
    # Optional hold time before auto-disable (cv.positive_time_period_milliseconds returns milliseconds)
    if CONF_POST_ARRIVAL_HOLD_MS in config:
        cg.add(var.set_post_arrival_hold_ms(config[CONF_POST_ARRIVAL_HOLD_MS]))
    
    await cg.register_component(var, config)
    await stepper.register_stepper(var, preprocessed_config)
    await modbus.register_modbus_device(var, config)



# ============================================================================
# Actions - using stepper.* namespace for ESPHome compatibility
# ============================================================================

CONF_ENABLE = "enable"
CONF_DIRECTION = "direction"
CONF_MODE = "mode"
CONF_CURRENT = "current"
CONF_RPM = "rpm"
CONF_PERCENT = "percent"
CONF_SUBDIVISION = "subdivision"

# stepper.enable
@automation.register_action(
    "stepper.enable",
    EnableMotorAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_enable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.disable
@automation.register_action(
    "stepper.disable",
    DisableMotorAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_disable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.run_continuous
@automation.register_action(
    "stepper.run_continuous",
    RunContinuousAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_SPEED): cv.templatable(validate_speed),  # Flexible units
        cv.Optional(CONF_DIRECTION, default="CW"): cv.templatable(cv.enum(HOMING_DIRECTIONS, upper=True)),
    }),
)
async def stepper_run_continuous_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    # Speed can be templatable, but validation happens at runtime
    # For now, we accept the speed value and let C++ code handle conversion
    speed_config = config[CONF_SPEED]
    if isinstance(speed_config, dict):
        # Static value - convert to steps/s at compile time
        parent_config = await cg.get_variable(config[CONF_ID])
        # TODO: Get steps_per_revolution from parent and convert
        # For now, pass the raw value and type to C++
        template_ = await cg.templatable(speed_config["value"], args, cg.float_)
        cg.add(var.set_speed(template_))
    else:
        # Templatable value - pass as-is
        template_ = await cg.templatable(speed_config, args, cg.float_)
        cg.add(var.set_speed(template_))
    
    template_ = await cg.templatable(config[CONF_DIRECTION], args, cg.uint8)
    cg.add(var.set_direction(template_))
    return var

# stepper.stop
@automation.register_action(
    "stepper.stop",
    StopMotorAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.emergency_stop
@automation.register_action(
    "stepper.emergency_stop",
    EmergencyStopAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_emergency_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.home - uses virtual or real homing based on config
@automation.register_action(
    "stepper.home",
    HomeAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_home_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.reset_position - software baseline reset
@automation.register_action(
    "stepper.reset_position",
    ResetPositionAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_reset_position_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.calibrate
@automation.register_action(
    "stepper.calibrate",
    CalibrateMotorAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_calibrate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.release_protection
@automation.register_action(
    "stepper.release_protection",
    ReleaseProtectionAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_release_protection_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.restart
@automation.register_action(
    "stepper.restart",
    RestartMotorAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_restart_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.set_work_mode
@automation.register_action(
    "stepper.set_work_mode",
    SetWorkModeAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_MODE): cv.templatable(cv.enum(WORK_MODES, upper=True)),
    }),
)
async def stepper_set_work_mode_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_MODE], args, cg.uint16)
    cg.add(var.set_mode(template_))
    return var

# stepper.set_working_current
@automation.register_action(
    "stepper.set_working_current",
    SetWorkingCurrentAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_CURRENT): cv.templatable(cv.int_range(min=0, max=5200)),
    }),
)
async def stepper_set_working_current_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_CURRENT], args, cg.uint16)
    cg.add(var.set_current(template_))
    return var

# stepper.set_holding_current_percent
@automation.register_action(
    "stepper.set_holding_current_percent",
    SetHoldingCurrentPercentAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_PERCENT): cv.templatable(cv.int_range(min=10, max=90)),
    }),
)
async def stepper_set_holding_current_percent_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_PERCENT], args, cg.uint8)
    cg.add(var.set_percent(template_))
    return var

# stepper.set_microstepping
@automation.register_action(
    "stepper.set_microstepping",
    SetMicrosteppingAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(Servo42dRs485),
        cv.Required(CONF_SUBDIVISION): cv.templatable(cv.int_range(min=1, max=256)),
    }),
)
async def stepper_set_microstepping_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    
    template_ = await cg.templatable(config[CONF_SUBDIVISION], args, cg.uint16)
    cg.add(var.set_subdivision(template_))
    return var

# stepper.key_lock
@automation.register_action(
    "stepper.key_lock",
    KeyLockAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_key_lock_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.key_unlock
@automation.register_action(
    "stepper.key_unlock",
    KeyUnlockAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
        },
        key=CONF_ID,
    ),
)
async def stepper_key_unlock_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# stepper.set_target - Override ESPHome default to call our method
@automation.register_action(
    "stepper.set_target",
    SetTargetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
            cv.Required(CONF_TARGET): cv.templatable(cv.int_),
        }
    ),
)
async def stepper_set_target_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_TARGET], args, int)
    cg.add(var.set_target(template_))
    return var

# stepper.report_position - Override ESPHome default to call our method
@automation.register_action(
    "stepper.report_position",
    ReportPositionAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Servo42dRs485),
            cv.Required(CONF_POSITION): cv.templatable(cv.int_),
        }
    ),
)
async def stepper_report_position_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_POSITION], args, int)
    cg.add(var.set_position(template_))
    return var


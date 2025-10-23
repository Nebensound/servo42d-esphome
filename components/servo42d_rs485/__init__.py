"""
Servo42D RS485 Component for ESPHome
Supports MKS Servo42D/57D closed-loop stepper motors via MODBUS-RTU
"""

CODEOWNERS = ["@jowgn"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["modbus"]

# The actual implementation is in the stepper subcomponent
# This allows the component to be used as: stepper.servo42d_rs485
# which internally uses the stepper platform with MODBUS-RTU communication

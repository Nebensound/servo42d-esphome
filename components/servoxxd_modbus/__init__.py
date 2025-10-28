"""
ServoXXD Modbus Component for ESPHome
Supports MKS ServoXXD (28D/35D/42D/57D) closed-loop stepper motors via Modbus RTU
"""

CODEOWNERS = ["@jowgn"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["modbus"]

# The actual implementation is in the stepper subcomponent
# This allows the component to be used as: stepper.servoxxd_modbus
# which internally uses the stepper platform with Modbus RTU communication

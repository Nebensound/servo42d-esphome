# Layer 4: Transport Layer (Modbus) - Detailed Specification

**Parent Document:** [02-cpp-interface.md](./02-cpp-interface.md)  
**Status:** 🔵 SPECIFICATION – Layer 4 implementation details

**Navigation:**

- [← Previous: Layer 3 (CommandQueue)](./02c-layer3-command-queue.md)
- [← Back to Overview](./02-cpp-interface.md#layer-4-transport-layer-modbus-specific)

---

## Overview

**Files:** `servoxxd_modbus.h` / `servoxxd_modbus.cpp`

**Design Pattern:** Command Pattern + State Machine

**Role in Architecture:**

- Provides Modbus-specific command classes and protocol handling
- Encapsulates Modbus register encoding/decoding
- Manages command lifecycle (PENDING → EXECUTING → COMPLETED/FAILED/TIMEOUT)
- Can be replaced with other transport layers (e.g., CAN, UART)

## Command State Machine

### States

```cpp
enum class CommandState {
  PENDING,    // Command in queue, not yet sent
  EXECUTING,  // Command sent to hardware, waiting for response
  COMPLETED,  // Response received and processed successfully
  FAILED,     // Modbus error received (exception code)
  TIMEOUT     // No response within timeout period
};
```

### State Transitions

| From → To | Trigger | Condition | Action |
|-----------|---------|-----------|--------|
| (created) → PENDING | enqueue() | - | Add command to queue |
| PENDING → EXECUTING | execute_next() | is_executing_ == false | Send Modbus request, start timeout timer, set is_executing_ = true |
| EXECUTING → COMPLETED | on_response_received() | Valid response data | Process response data, call completion callback(success=true), set is_executing_ = false |
| EXECUTING → FAILED | on_error_received() | Modbus exception code | Log error details, call completion callback(success=false), set is_executing_ = false |
| EXECUTING → TIMEOUT | check_timeout() | Elapsed time > timeout | Log timeout, call completion callback(success=false), set is_executing_ = false |
| COMPLETED/FAILED/TIMEOUT → (deleted) | execute_next() | Command processed | Remove from queue head, call execute_next() for next command |

### Event Processing

- **enqueue()**: Add command to queue tail in PENDING state
- **execute_next()**: If no command executing and queue not empty, transition first PENDING command to EXECUTING
- **on_response_received()**: Complete current EXECUTING command → COMPLETED, free execution slot, process next
- **on_error_received()**: Fail current EXECUTING command → FAILED, free execution slot, process next
- **check_timeout()**: Called periodically in loop(), detects stuck commands → TIMEOUT, free execution slot, process next

### Callbacks

- **Completion callback:** `std::function<void(bool success)>` - called when command reaches terminal state (COMPLETED, FAILED, or TIMEOUT)
- **Data callback (optional):** `std::function<void(const std::vector<uint8_t>&)>` - called on successful response with parsed data

### State Machine Invariants

1. Only one command can be in EXECUTING state at any time (single-flight guarantee from Layer 3)
2. Commands in PENDING state remain in queue until their turn
3. Terminal states (COMPLETED/FAILED/TIMEOUT) are immediately removed from queue
4. is_executing_ flag acts as mutex - prevents command interleaving

## Command Classes

### BaseCommand (abstract base)

**Metadata:**

- Register address
- Timeout duration

**State:**

- CommandState (PENDING → EXECUTING → COMPLETED | FAILED | TIMEOUT)

**Callbacks:**

- completion(success)
- optional data callback

**Virtual Methods:**

- `execute(device)` - Send Modbus request
- `process_response(data)` - Parse and validate response
- `get_function_code()` - Return Modbus function code
- `get_command_name()` - Return human-readable name for logging

### ReadCommand (0x04 - Read Input Registers)

**Constructor:** `ReadCommand(register_address, quantity)`

**Functionality:**

- Reads multiple input registers (Modbus function code 0x04)
- Validates response contains `quantity × 2` bytes
- Extracts register values, calls data callback

**Example:**

```cpp
auto cmd = std::make_unique<ReadCommand>(0x30, 2);  // Read encoder position (2 registers)
cmd->set_data_callback([](const std::vector<uint8_t>& data) {
    uint32_t position = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    ESP_LOGD(TAG, "Encoder position: %u", position);
});
queue->enqueue(std::move(cmd));
```

### WriteCommand (0x06 - Write Single Register)

**Constructor:** `WriteCommand(register_address, value)`

**Functionality:**

- Writes single register (Modbus function code 0x06)
- Validates response echoes register address + value
- Confirms write success

**Example:**

```cpp
auto cmd = std::make_unique<WriteCommand>(0xFD, mode_value);  // Set work mode
cmd->set_completion_callback([](bool success) {
    if (success) {
        ESP_LOGD(TAG, "Work mode set successfully");
    }
});
queue->enqueue(std::move(cmd));
```

### MultiWriteCommand (0x10 - Write Multiple Registers)

**Constructor:** `MultiWriteCommand(register_address, values)`

**Functionality:**

- Writes multiple consecutive registers (Modbus function code 0x10)
- Validates response confirms register address + quantity written
- Used for multi-register commands (e.g., position + speed + acceleration)

**Example:**

```cpp
std::vector<uint16_t> values = {position_high, position_low, speed_value, accel_value};
auto cmd = std::make_unique<MultiWriteCommand>(0x00, values);  // Move to position
cmd->set_completion_callback([](bool success) {
    if (success) {
        ESP_LOGD(TAG, "Move command sent successfully");
    }
});
queue->enqueue(std::move(cmd));
```

## Modbus Protocol Details

### Function Codes

- **0x04**: Read Input Registers (most status reads)
- **0x06**: Write Single Register (simple commands)
- **0x10**: Write Multiple Registers (complex commands)

### Error Handling

- **Exception Codes**: Logged with function code and exception code
- **Timeouts**: Detected by Layer 3 (CommandQueue), command marked as TIMEOUT
- **Invalid Responses**: Logged, command marked as FAILED

### Register Encoding

- **Position**: Split format (2 registers: high + low word)
- **Speed**: Single register (RPM or steps/sec, depends on mode)
- **Acceleration**: Single register (RPM/s or steps/sec², depends on mode)
- **Status**: Bitfield (enabled, protection, homing, etc.)

## Benefits

- **Encapsulation**: All Modbus details isolated in Layer 4
- **Extensibility**: Easy to add new command types or replace with other transport
- **Reliability**: State machine ensures proper command lifecycle
- **Testability**: Command classes can be unit tested independently

---

**Navigation:**

- [← Previous: Layer 3 (CommandQueue)](./02c-layer3-command-queue.md)
- [← Back to Overview](./02-cpp-interface.md#layer-4-transport-layer-modbus-specific)

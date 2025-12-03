# Makefile for ServoXxd Unit Tests

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I../.. -I. -I../../components/servoxxd/stepper
LDFLAGS =

# Source files
COMPONENT_DIR = ../../components/servoxxd/stepper
MOCK_DIR = esphome/core

SOURCES = \
	$(COMPONENT_DIR)/servoxxd.cpp \
	$(COMPONENT_DIR)/servoxxd_modbus.cpp \
	$(COMPONENT_DIR)/servoxxd_stepper_engine.cpp \
	$(COMPONENT_DIR)/servoxxd_command_queue.cpp \
	$(COMPONENT_DIR)/servoxxd_speed.cpp \
	$(COMPONENT_DIR)/servoxxd_acceleration.cpp \
	$(COMPONENT_DIR)/servoxxd_position.cpp \
	$(MOCK_DIR)/hal.cpp

QUEUE_SOURCES = \
	$(COMPONENT_DIR)/servoxxd_command_queue.cpp

ENGINE_SOURCES = \
	$(COMPONENT_DIR)/servoxxd.cpp \
	$(COMPONENT_DIR)/servoxxd_stepper_engine.cpp \
	$(COMPONENT_DIR)/servoxxd_command_queue.cpp \
	$(COMPONENT_DIR)/servoxxd_modbus.cpp \
	$(COMPONENT_DIR)/servoxxd_speed.cpp \
	$(COMPONENT_DIR)/servoxxd_acceleration.cpp \
	$(COMPONENT_DIR)/servoxxd_position.cpp

# Test executables (all active tests)
TESTS = test_speed test_acceleration test_position test_command_queue test_stepper_engine test_command_codec

# Colors for output
GREEN = \033[0;32m
RED = \033[0;31m
NC = \033[0m # No Color

.PHONY: all clean run test

all: $(TESTS)

# Build test executables
test_speed: test_speed.cpp $(SOURCES)
	@echo "Building $@..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_acceleration: test_acceleration.cpp $(SOURCES)
	@echo "Building $@..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_position: test_position.cpp $(SOURCES)
	@echo "Building $@..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_command_queue: test_command_queue.cpp $(QUEUE_SOURCES)
	@echo "Building $@..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_stepper_engine: test_stepper_engine.cpp $(ENGINE_SOURCES)
	@echo "Building $@..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_command_codec: test_command_codec.cpp
	@echo "Building $@..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Run all tests
test: $(TESTS)
	@echo ""
	@echo "$(GREEN)========================================$(NC)"
	@echo "$(GREEN)Running All Unit Tests$(NC)"
	@echo "$(GREEN)========================================$(NC)"
	@echo ""
	@for test in $(TESTS); do \
		echo "$(GREEN)Running $$test...$(NC)"; \
		echo ""; \
		./$$test || exit 1; \
		echo ""; \
	done
	@echo "$(GREEN)========================================$(NC)"
	@echo "$(GREEN)✅ All Tests Passed!$(NC)"
	@echo "$(GREEN)========================================$(NC)"
	@echo ""

# Run individual tests
run: $(TESTS)
	@./test_speed
	@./test_acceleration
	@./test_position

# Clean build artifacts
clean:
	@echo "Cleaning..."
	rm -f $(TESTS)
	@echo "Done."

# Help
help:
	@echo "ServoXxd Unit Tests Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  all              Build all test executables"
	@echo "  test             Build and run all tests"
	@echo "Individual tests:"
	@echo "  test_speed       Build Speed class tests"
	@echo "  test_acceleration Build Acceleration class tests"
	@echo "  test_position    Build Position class tests"
	@echo "  test_command_queue Build CommandQueue tests"
	@echo "  test_stepper_engine Build StepperEngine tests"
	@echo "  test_command_codec Build CommandCodec tests"
	@echo "Individual tests:"
	@echo "  test_speed       Build Speed class tests"
	@echo "  test_acceleration Build Acceleration class tests"
	@echo "  test_position    Build Position class tests"

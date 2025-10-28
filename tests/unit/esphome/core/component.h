#pragma once

// Mock ESPHome component header for unit testing

namespace esphome
{

  class Component
  {
  public:
    virtual ~Component() = default;

    // Virtual methods that can be overridden
    virtual void setup() {}
    virtual void loop() {}
    virtual void dump_config() {}
  };

} // namespace esphome

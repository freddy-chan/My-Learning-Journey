# 🛠️ Build Instructions and Setup Guide

## Prerequisites

### Software Requirements
1. **ESP-IDF v5.0 or later**
   - Download from: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
   - Follow the complete installation guide for your platform

2. **Development Environment** (Choose one):
   - **VS Code** with ESP-IDF extension (Recommended)
   - **ESP-IDF Command Line** with any text editor
   - **Eclipse** with ESP-IDF plugin

### Hardware Requirements
- **ESP32 Development Board** (ESP32-DevKitC, NodeMCU-32S, or similar)
- **USB Cable** for programming and power
- **Components for each phase** (see Hardware Setup section)

---

## 🚀 Quick Start

### 1. Environment Setup
```bash
# Clone or navigate to project directory
cd practice-project

# Set ESP32 as target (first time only)
idf.py set-target esp32

# Configure project settings
idf.py menuconfig
```

### 2. Configuration
In menuconfig, navigate to:
- **Smart Environment Data Logger Configuration**
  - Configure GPIO pins for your hardware setup
  - Set WiFi credentials (for later phases)
  - Adjust sensor reading intervals

### 3. Build and Flash
```bash
# Build the project
idf.py build

# Flash to ESP32 (connect your board first)
idf.py flash

# Monitor serial output
idf.py monitor

# Or combine flash and monitor
idf.py flash monitor
```

### 4. Exit Monitor
Press `Ctrl+]` to exit the serial monitor.

---

## 📅 Phase-by-Phase Setup

### Phase 1: Basic GPIO (Days 1-7)
**Hardware Needed:**
- ESP32 board
- LED (if not using built-in)
- Push button
- 220Ω resistor for LED
- 10kΩ resistor for button pull-up
- Breadboard and jumper wires

**Connections:**
```
ESP32 Pin    Component
─────────────────────────
GPIO 2       LED Anode (+)
GND          LED Cathode (-) via 220Ω resistor
GPIO 0       Button (one side)
3.3V         Button (other side) via 10kΩ resistor
GND          Button (other side) direct
```

**Test Commands:**
```bash
# Build and run
idf.py flash monitor

# Expected output: LED blinking demos
# Press button to test interrupt (Phase 1 Day 3-4)
```

### Phase 2: Sensors (Days 8-14)
**Additional Hardware:**
- DHT22 temperature/humidity sensor
- MPU6050 accelerometer/gyroscope
- 4.7kΩ resistor for DHT22 pull-up

**Connections:**
```
DHT22:
  VCC  → 3.3V
  GND  → GND  
  DATA → GPIO 4 (via 4.7kΩ to 3.3V)

MPU6050:
  VCC → 3.3V
  GND → GND
  SDA → GPIO 21
  SCL → GPIO 22
```

### Phase 3: Storage & Network (Days 15-21)
**Additional Hardware:**
- SD card module (SPI interface)
- MicroSD card (formatted as FAT32)

**Connections:**
```
SD Card Module:
  VCC → 3.3V
  GND → GND
  CS  → GPIO 5
  MOSI→ GPIO 23
  MISO→ GPIO 19
  CLK → GPIO 18
```

### Phase 4: Advanced Features (Days 22-28)
**Software Only:**
- OTA updates via WiFi
- FreeRTOS task management
- Production features

---

## 🔧 Development Workflow

### Daily Development Cycle
1. **Read the day's objectives** in `LEARNING_ROADMAP.md`
2. **Implement the required features** following the guide
3. **Test the implementation** with `idf.py flash monitor`
4. **Debug any issues** using ESP-IDF logging
5. **Document your learning** in comments or notes

### Common Development Commands
```bash
# Clean build (if having issues)
idf.py fullclean
idf.py build

# Size analysis
idf.py size

# Component-specific build
idf.py build --component led_controller

# Only flash app (faster for development)
idf.py app-flash monitor
```

### Debugging Tips
1. **Use ESP_LOG macros** for debugging:
   ```c
   ESP_LOGI(TAG, \"Debug info: %d\", value);
   ESP_LOGE(TAG, \"Error: %s\", esp_err_to_name(ret));
   ```

2. **Check log levels** in menuconfig:
   - Component config → Log output → Default log verbosity

3. **Monitor heap usage**:
   ```c
   ESP_LOGI(TAG, \"Free heap: %d bytes\", esp_get_free_heap_size());
   ```

---

## 📁 Project Structure Explained

```
project/
├── components/                 # Custom ESP-IDF components
│   ├── led_controller/        # Phase 1: GPIO control
│   ├── sensor_dht22/          # Phase 2: Temperature sensor
│   ├── sensor_mpu6050/        # Phase 2: Motion sensor  
│   ├── storage_manager/       # Phase 3: SD card operations
│   ├── wifi_manager/          # Phase 3: WiFi connectivity
│   ├── mqtt_client/           # Phase 3: Cloud communication
│   └── ota_manager/           # Phase 4: Over-air updates
├── main/
│   ├── CMakeLists.txt         # Main component config
│   └── main.c                 # Application entry point
├── CMakeLists.txt             # Project configuration
├── Kconfig.projbuild          # Menuconfig options
├── sdkconfig                  # Generated configuration
├── README.md                  # Project overview
├── LEARNING_ROADMAP.md        # 4-week learning guide
└── BUILD_INSTRUCTIONS.md      # This file
```

### Component Structure
Each component follows ESP-IDF conventions:
```
component_name/
├── CMakeLists.txt            # Build configuration
├── include/
│   └── component_name.h      # Public API header
└── component_name.c          # Implementation
```

---

## 🐛 Troubleshooting

### Common Issues

**Build Errors:**
```bash
# Missing ESP-IDF environment
source ~/esp/esp-idf/export.sh  # Linux/Mac
%userprofile%\\esp\\esp-idf\\export.bat  # Windows

# Component not found
# Check CMakeLists.txt in main/ includes component

# Permission denied (Linux/Mac)
sudo chmod 666 /dev/ttyUSB0  # or ttyACM0
```

**Flash/Upload Issues:**
```bash
# Wrong port
idf.py -p /dev/ttyUSB0 flash  # Linux
idf.py -p COM3 flash          # Windows

# Board not in download mode
# Hold BOOT button while pressing EN/RST

# Bootloader issues
idf.py erase-flash
idf.py flash
```

**Runtime Issues:**
```bash
# Check connections with multimeter
# Verify power supply (3.3V, not 5V for sensors)
# Review GPIO assignments in menuconfig
# Check log levels for detailed output
```

### Advanced Debugging
```bash
# Enable JTAG debugging (if supported)
idf.py openocd

# Core dump analysis
idf.py coredump-info

# Performance monitoring
idf.py monitor --print_filter=\"*:I\"
```

---

## 📊 Project Milestones

### Week 1 Checkpoints
- [ ] LED blinks using GPIO API
- [ ] Button interrupt working
- [ ] Menuconfig options functional
- [ ] Component structure established

### Week 2 Checkpoints  
- [ ] DHT22 reading temperature/humidity
- [ ] MPU6050 providing motion data
- [ ] Unified sensor data structure
- [ ] Error handling implemented

### Week 3 Checkpoints
- [ ] SD card logging sensor data
- [ ] WiFi connection established
- [ ] MQTT publishing to broker
- [ ] Data persistence working

### Week 4 Checkpoints
- [ ] Multi-task architecture running
- [ ] OTA updates functional
- [ ] Production error handling
- [ ] System monitoring active

---

## 🔗 Useful Resources

### Documentation
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)

### Component Examples
- [ESP-IDF Examples](https://github.com/espressif/esp-idf/tree/master/examples)
- [ESP Component Registry](https://components.espressif.com/)

### Debugging Tools
- [ESP-IDF Monitor](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-monitor.html)
- [ESP32 Exception Decoder](https://github.com/me-no-dev/EspExceptionDecoder)

---

## 💡 Tips for Success

1. **Start Simple**: Get each phase working before moving to the next
2. **Read Documentation**: ESP-IDF has excellent docs - use them!
3. **Test Frequently**: Build and test after each small change
4. **Use Version Control**: Commit working versions before major changes
5. **Ask Questions**: ESP32 community is very helpful
6. **Measure Progress**: Check off milestones as you complete them

**Ready to start building?** Begin with [Day 1](LEARNING_ROADMAP.md#day-1-2-esp-idf-fundamentals) of the learning roadmap!
# DHT22 Integration Build and Test Guide

## Build Instructions

### Prerequisites
- ESP-IDF v5.5 installed and configured
- ESP32 development board
- DHT22 sensor
- LED and push button (if not using built-in components)

### Setup ESP-IDF Environment
```bash
# Windows PowerShell
C:\Espressif\frameworks\esp-idf-v5.5\export.ps1

# Linux/macOS
. $HOME/esp/esp-idf/export.sh
```

### Build Commands
```bash
# Navigate to project directory
cd "c:\Users\ACER\Desktop\My-Learning-Journey\ESP-IDF\practice-project"

# Clean previous build (if needed)
idf.py fullclean

# Configure the project (optional)
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor
```

## Hardware Connections

### DHT22 Sensor
- VCC → 3.3V or 5V
- GND → GND  
- DATA → GPIO4

### LED (if external)
- Anode → GPIO2
- Cathode → GND (through 220Ω resistor)

### Button (if external)
- One terminal → GPIO0
- Other terminal → GND
- Internal pull-up enabled in software

## Expected Output

When the application runs successfully, you should see:

```
I (xxx) MAIN_APP: === Smart Environment Data Logger Started ===
I (xxx) MAIN_APP: Day 8-10: DHT22 Temperature/Humidity Sensor Integration
I (xxx) MAIN_APP: LED controller initialized
I (xxx) MAIN_APP: Button controller initialized with interrupt support
I (xxx) MAIN_APP: DHT22 sensor initialized on GPIO 4
I (xxx) MAIN_APP: System health monitoring started
I (xxx) MAIN_APP: === Usage Instructions ===
I (xxx) MAIN_APP: - Single Click: Toggle auto LED mode
I (xxx) MAIN_APP: - Double Click: Change blink pattern (0-3)
I (xxx) MAIN_APP: - Long Press (1s): Reset statistics and show sensor info
I (xxx) MAIN_APP: - Hold Button: LED stays on while pressed
I (xxx) MAIN_APP: - Automatic: DHT22 readings every 5 seconds with LED patterns
I (xxx) MAIN_APP: Waiting for DHT22 sensor to stabilize...
I (xxx) MAIN_APP: === Sensor Reading 1 ===
I (xxx) MAIN_APP: Environmental Data:
I (xxx) MAIN_APP:   Temperature: 23.5°C (74.3°F)
I (xxx) MAIN_APP:   Humidity: 65.2%
I (xxx) MAIN_APP:   Heat Index: 24.1°C
I (xxx) MAIN_APP:   Dew Point: 16.8°C
```

## LED Pattern Indicators

| Temperature Range | LED Pattern | Description |
|------------------|-------------|-------------|
| < 20°C | Slow blink (1s on/off) | Cold temperature |
| 20-25°C | Normal blink (500ms on/off) | Comfortable temperature |
| > 25°C | Fast blink (200ms on/off) | Warm/hot temperature |
| Sensor Error | Double blink | DHT22 communication error |

## Button Controls

- **Single Click**: Toggle automatic LED mode on/off
- **Double Click**: Change LED blink pattern (0-3)
- **Long Press (1s)**: Reset statistics and show sensor information
- **Hold Button**: LED stays on while pressed (manual override)

## Troubleshooting

### Build Errors
1. **"idf.py not recognized"**: Run ESP-IDF export script first
2. **GPIO conflicts**: Check pin assignments in pin definitions
3. **Component not found**: Verify all component directories have CMakeLists.txt

### Runtime Issues
1. **DHT22 not responding**: Check wiring and power supply
2. **Invalid readings**: Ensure 2-second minimum interval between readings
3. **LED not working**: Check GPIO pin assignment and active level

### Common Solutions
- Ensure ESP-IDF environment is properly set up
- Check hardware connections
- Verify sensor power supply (3.3V or 5V)
- Use shorter wires for DHT22 data line
- Add 10kΩ pull-up resistor to DHT22 data line if needed

## Performance Characteristics

- **Sensor Reading Interval**: 5 seconds (configurable)
- **DHT22 Accuracy**: ±0.5°C temperature, ±2-5% RH humidity  
- **Memory Usage**: ~15KB flash, ~2KB RAM
- **Power Consumption**: ~100mA active, depends on LED usage

## Next Steps

After successful DHT22 integration:
1. Day 11-12: Add MPU6050 accelerometer/gyroscope
2. Day 13-14: Sensor data integration and display
3. Week 2: Storage and communication features
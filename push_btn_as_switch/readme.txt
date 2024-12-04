# ESP32 Button Toggle Project

This project demonstrates how to use a push button to toggle the built-in LED on an ESP32 using the Arduino framework. It includes debouncing logic to ensure stable input handling from the button.

---

## **Features**
- **Push Button Control**: A physical button toggles the LED on/off.
- **Debouncing**: Handles button bouncing for reliable operation.
- **Versatility**: Can be adapted for other GPIO pins or extended to additional functionality.

---

## **Applications**
This project is a fundamental building block for:
- User interface controls in embedded systems.
- IoT applications such as device mode toggling, configuration changes, or state indicators.
- Prototyping, debugging, or educational purposes.

---

## **Hardware Requirements**
1. **ESP32 Development Board**
2. **Push Button**
3. Optional: External pull-up resistor (if not using the internal pull-up).

---

## **Circuit Diagram**
1. **Button Connection**:
   - One terminal of the button to GPIO0 (defined as `BUTTON_PIN` in the code).
   - The other terminal to GND.
   - If using an external pull-up, connect a resistor (e.g., 10kΩ) between GPIO0 and 3.3V.

2. **Built-in LED**:
   - Most ESP32 boards have a built-in LED connected to GPIO2 (`LED_PIN` in the code). No additional wiring is required.

---

## **Software Requirements**
- **Arduino IDE**:
  - Install the ESP32 Board Package in the Arduino IDE.
  - Install necessary libraries (if applicable).

---

## **Setup**
1. Clone this repository or copy the code into the Arduino IDE.
2. Connect the hardware as described in the circuit diagram.
3. Compile and upload the code to the ESP32 board.

---

## **Code Description**
The logic in the code includes:
1. **Initialization**:
   - Sets up the button pin as an input with an internal pull-up resistor.
   - Sets the LED pin as an output.
   - Initializes the LED state to `LOW`.

2. **Debouncing Logic**:
   - Ensures stable button input by implementing a 50ms debounce delay.

3. **Toggling**:
   - When the button is pressed and released (detected via state change), the LED toggles between ON and OFF states.

---

## **Usage**
1. Power up the ESP32.
2. Press the button to toggle the LED.
3. Monitor the serial output (optional) to see the state changes.

---

## **Potential Extensions**
- Use multiple buttons to control multiple LEDs or devices.
- Add MQTT/HTTP functionality to control the LED remotely.
- Replace the LED control with another action (e.g., sending data to a cloud server).

---

## **License**
This project is licensed under the MIT License. Feel free to use and modify the code.

---

## **Contributors**
- **[Your Name]** - Developer


# ESP32 LED and Button Control with FreeRTOS

This project demonstrates how to control an LED using a button on an ESP32 microcontroller with FreeRTOS and GPIO interrupts.

## Table of Contents
- [Overview](#overview)
- [Dependencies](#dependencies)
- [Pin Definitions](#pin-definitions)
- [Global Variables](#global-variables)
- [Functions](#functions)
- [Usage](#usage)
- [Notes](#notes)

## Overview
The application toggles the LED state with a button press. The button press triggers an interrupt, which is handled by FreeRTOS tasks.

## Dependencies
- `driver/gpio.h`: Provides GPIO configuration and control.
- `freertos/FreeRTOS.h`: The FreeRTOS operating system API.
- `freertos/task.h`: FreeRTOS task management.
- `freertos/queue.h`: FreeRTOS queue management.

## Pin Definitions
- **LED_PIN**: GPIO pin connected to the LED (Pin 14).
- **BUTTON_PIN**: GPIO pin connected to the button (Pin 13).

## Global Variables
- `gpio_evt_queue`: A queue handle for storing GPIO events.
- `led_state`: A boolean indicating the current state of the LED (on/off).
- `should_blink`: A boolean indicating whether the LED should blink.

## Functions

### gpio_isr_handler
```c
static void IRAM_ATTR gpio_isr_handler(void *arg)
```
Interrupt Service Handler for GPIO. It sends the GPIO number to the queue when an interrupt is detected.
- **Parameters**: `arg` - GPIO number (casted to `uint32_t`).
- **Returns**: None.

### led_task
```c
static void led_task(void *arg)
```
A FreeRTOS task that controls the LED. It toggles the LED state every 500 milliseconds if `should_blink` is `true`.
- **Parameters**: `arg` - Task parameter (not used).
- **Returns**: None.

### button_task
```c
static void button_task(void *arg)
```
A FreeRTOS task that handles button press events. It receives the GPIO number from the queue, debounces the button, and toggles `should_blink`.
- **Parameters**: `arg` - Task parameter (not used).
- **Returns**: None.

### app_main
```c
void app_main(void)
```
The main function that sets up the GPIO, creates the tasks, and installs the interrupt service.
- **Creates Queue**: Initializes the queue for GPIO events.
- **Configures GPIO**: Sets up GPIO pins for the LED and button.
- **Installs ISR**: Adds an interrupt service routine for the button GPIO.
- **Creates Tasks**: Creates the LED and button tasks.

## Usage
- **Compile and Flash**: Compile the code and flash it to the ESP32.
- **Button Press**: Press the button to toggle the blinking state of the LED.

## Notes
- Ensure proper debouncing to avoid false triggers.
- Adjust the task priorities and stack sizes as necessary for your application.



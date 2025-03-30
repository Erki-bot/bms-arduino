# Arduino Battery Management System (BMS)

This project is an Arduino-based Battery Management System (BMS) that monitors battery voltage, calculates battery percentage, and manages charging operations. It includes features such as an LCD display, MQTT communication, and Bluetooth-based manual control.

## Features

1. **Battery Monitoring**:

- Reads battery voltage using an analog pin.
- Calculates battery percentage based on predefined voltage thresholds.

2. **LCD Display**:
  - Displays the current battery voltage and percentage.

3. **MQTT Communication**:
  - Publishes battery voltage and percentage to an MQTT broker.
  - Subscribes to commands for controlling the charging system.

4. **Bluetooth Control**:
  - Allows manual control of the charging system via Bluetooth commands.

5. **Automatic Charging Control**:
  - Disables charging when the battery is fully charged (voltage >= 14.4V).
  - Enables charging when the battery percentage is less than or equal to 15%.

6. **Wi-Fi Connectivity**:
  - Connects to a Wi-Fi network for MQTT communication.
  - Includes a reconnect mechanism for maintaining Wi-Fi connectivity.

7. **LED Indicators**:
  - Indicates Wi-Fi and MQTT connection status using LEDs.

## Main Loop Functionality

The main loop performs the following tasks:

1. Reads the battery voltage and calculates the battery percentage.
2. Updates the LCD display with the current voltage and battery percentage.
3. Publishes the voltage and battery percentage to an MQTT topic if the client is connected.
4. Processes Bluetooth commands to manually enable or disable charging, or switch to automatic mode.
5. Implements automatic charging control based on battery voltage and percentage.

### Automatic Mode Behavior

- **Charging Disabled**: When the battery is fully charged (voltage >= 14.4V).
- **Charging Enabled**: When the battery percentage is less than or equal to 15%.

### Manual Mode Behavior

- **Enable Charging**: Bluetooth command `1`.
- **Disable Charging**: Bluetooth command `0`.
- **Switch to Automatic Mode**: Bluetooth command `3`.

## Hardware Requirements

- Arduino board (e.g., ESP32).
- LCD with I2C interface.
- Wi-Fi module (built-in for ESP32).
- Bluetooth module (e.g., HC-05).
- Voltage divider circuit for battery voltage measurement.
- LEDs for Wi-Fi and MQTT status indicators.

## Software Requirements

- Arduino IDE.
- Libraries:
  - `Wire.h`
  - `LiquidCrystal_I2C.h`
  - `WiFi.h`
  - `PubSubClient.h`

## Wiring Diagram

- **Battery Voltage Measurement**: Connect the battery to the analog pin through a voltage divider circuit.
- **LCD**: Connect the LCD to the I2C pins (SDA and SCL).
- **Wi-Fi and Bluetooth**: Use the appropriate pins for communication.

## Usage

1. Configure the Wi-Fi credentials and MQTT broker details in the code.
2. Upload the code to the Arduino board.
3. Monitor the battery status on the LCD display.
4. Use an MQTT client to send commands or monitor data.
5. Use a Bluetooth terminal to send manual control commands.

## Bluetooth Commands

- `1`: Enable charging (manual mode).
- `0`: Disable charging (manual mode).
- `3`: Switch to automatic mode.

## MQTT Topics

- **Data Topic**: Publishes battery voltage and percentage.
- **Command Topic**: Subscribes to commands for controlling the charging system.

## Notes

- Ensure the voltage divider circuit is correctly configured to avoid damaging the analog pin.
- Adjust the voltage thresholds (`minVoltage12V` and `maxVoltage12V`) based on your battery specifications.
- Use appropriate resistor values for the voltage divider to ensure accurate readings.

## License

This project is open-source and available under the MIT License.
---

# Autonomous Alarm System with PID Control

This Arduino-based project is an autonomous alarm system that utilizes PID (Proportional-Integral-Derivative) control to maneuver and respond to sensor inputs. The system integrates Firebase for remote control and monitoring.

## Features

- **Sensor Integration**: Utilizes IR sensors to detect position and trigger alarm response.
- **PID Control**: Implements PID algorithm to regulate motor speeds based on sensor readings.
- **Firebase Integration**: Allows remote control and monitoring through Firebase Realtime Database (RTDB).
- **Autonomous Behavior**: Self-regulates and responds to changes in sensor inputs, activating the alarm system as needed.

## Hardware Requirements

- Arduino board compatible with WiFi modules (e.g., ESP32, ESP8266)
- L298N motor driver
- Infrared (IR) sensors (quantity according to SENSOR_COUNT)
- Buzzer
- Power supply (e.g., 9V)

## Software Requirements

- Arduino IDE
- Libraries:
  - L298N
  - Firebase_ESP_Client
  - Add-ons: TokenHelper, RTDBHelper
## Simulation


https://github.com/the7ag/PID_LINE_FOLLOWER/assets/95578914/deb46ac8-b3ce-4c84-bb5f-69b51db9f42f



## Setup Instructions

1. Connect the hardware components according to the provided pin mappings in the code.
2. Open the Arduino IDE and install required libraries.
3. Copy the provided code into the Arduino IDE.
4. Replace placeholders (`Your_WiFi_SSID`, `Your_WiFi_Password`, `Your_Firebase_API_Key`, `Your_Firebase_RTDB_URL`) with your network credentials and Firebase details.
5. Upload the code to your Arduino board.

## Usage

1. Ensure the hardware setup is properly connected.
2. Power on the Arduino board.
3. The system will attempt to connect to the configured Wi-Fi network and Firebase.
4. Monitor the system's behavior via the serial monitor in the Arduino IDE.
5. Utilize the Firebase Realtime Database to control the system remotely by changing the `/play/play` key value:
   - `0`: Deactivates the alarm system.
   - `1`: Activates the alarm system.

## Troubleshooting

- Check the serial monitor for debugging information and error messages.
- Ensure proper connections and component configurations according to the provided pin mappings.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- Mention any contributors or references that assisted in the development of the project.

---

Feel free to modify and expand this README to provide more details, acknowledgments, or specific instructions tailored to your project's needs.

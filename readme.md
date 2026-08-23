# ESP32 Creality SpacePi Filament Dryer Controller

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; align-items: start; max-width: 100%;">
  <img 
    src="https://github.com/user-attachments/assets/35f24f5d-dcb9-4504-8d97-edbd4673cf3f" 
    alt="Thiết kế chưa có tên" 
    style="width: 100%; height: auto; border-radius: 8px; object-fit: contain;" 
  />
  <img 
    src="https://github.com/user-attachments/assets/ecca260b-d1b5-464c-8f04-cf6007d68fd9" 
    alt="image" 
    style="width: 100%; height: auto; border-radius: 8px; object-fit: contain;" 
  />
</div>

A complete custom firmware implementation for upgrading/controlling a 3D printer filament dryer (such as the Creality SpacePi) using an ESP32 Cheap Yellow Display (CYD 2.8" TFT + Touchscreen).

The firmware features a **Creality K1 OS-inspired dark dashboard**, precise PTC heater control via slow-PWM time-windowing, automatic temperature/humidity tracking via BME280, a full-featured async Web UI, Captive Portal Wi-Fi setup, and over-the-air (OTA) firmware flashing.

---

## Key Features

- **Creality K1 UI Style**: Custom dark-themed layout built with `TFT_eSPI` featuring real-time dual-series charts (Temperature & Humidity), status pills, and intuitive touch controls.
- **PTC Heater Control**: Slow-PWM time-windowed duty cycle control (200ms – 10000ms window size) designed for Opto-triac/SSR drivers (e.g., EL3063).
- **Dual Operating Modes**:
  - **Timer Mode**: Dry filament at target temperatures ($35^\circ\text{C} - 66^\circ\text{C}$) for preset or custom durations ($0.5\text{h} - 24\text{h}$).
  - **Auto Humidity Mode**: Continuously monitor chamber humidity and shut off automatically once target % RH is reached.
- **Safety Safeguards**:
  - Emergency thermal cut-off at $\ge 67^\circ\text{C}$.
  - Overheat cool-down logic with hysteresis.
  - Automatic $100\%$ exhaust fan purge until chamber drops below $40^\circ\text{C}$ upon stopping.
- **Web Dashboard**: Modern mobile-friendly web interface with live telemetry, filament profile selection (ABS / PETG / PLA), fan adjustments, and manual duty override.
- **Wi-Fi Captive Portal**: Built-in AP setup (`SpacePi_Setup` at `192.168.4.1`) for easy Wi-Fi configuration stored via ESP32 `Preferences` (NVS).
- **Web OTA Updates**: Flash newly compiled `.bin` binaries directly over Wi-Fi without USB cables.

---

## Hardware Pinout (CYD 2.8" ESP32-2432S028R)

| Function | ESP32 GPIO | Notes |
| :--- | :--- | :--- |
| **Heater Control (`PIN_HEATER`)** | GPIO 17 | Active LOW (Optocoupler / SSR driver) |
| **Fan PWM (`PIN_FAN`)** | GPIO 16 | 5kHz PWM MOSFET control |
| **I2C SDA** | GPIO 27 | CN1 Header Pin 3 (BME280) |
| **I2C SCL** | GPIO 22 | CN1 Header Pin 2 (BME280) |

---

## Required Libraries

Install the following libraries via the Arduino IDE Library Manager or PlatformIO:

- `TFT_eSPI` (configured for ST7789 / ILI9341 on CYD)
- `XPT2046_Touchscreen`
- `Adafruit BME280 Library`
- `Adafruit Unified Sensor`

---

## Getting Started

1. **Configure Display Settings**: Ensure your `TFT_eSPI` `User_Setup.h` matches the pin configuration and driver of your CYD board.
2. **Compile & Flash**: Open the `.ino` file in Arduino IDE or PlatformIO, select **ESP32 Dev Module**, and upload.
3. **Wi-Fi Provisioning**:
   - On first boot, the screen enters **WIFI SETUP MODE**.
   - Connect to the Wi-Fi AP: `SpacePi_Setup`.
   - Navigate to `http://192.168.4.1` in your browser, select your local Wi-Fi SSID, enter your password, and save.
4. **Access Web Interface**:
   - Connect to the same local network and open `http://spacepi.local` or the assigned IP address.
   - For OTA updates, visit `http://spacepi.local/update`.

---

## License

This project is licensed under the MIT License - feel free to use and modify for personal or commercial projects.

```

# ESP32 Creality SpacePi Filament Dryer Controller

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; align-items: start; max-width: 100%;">
  <img 
    src="https://github.com/user-attachments/assets/7e46bac3-b9eb-47a8-bc90-ba92987d3903"
    alt="Thiết kế chưa có tên" 
    style="width: 100%; height: auto; border-radius: 8px; object-fit: contain;" 
  />
  <img 
    src="https://github.com/user-attachments/assets/ecca260b-d1b5-464c-8f04-cf6007d68fd9" 
    alt="image" 
    style="width: 100%; height: auto; border-radius: 8px; object-fit: contain;" 
  />
</div>

A complete custom firmware implementation for upgrading and controlling a 3D printer filament dryer (such as the Creality SpacePi) using an ESP32 Cheap Yellow Display (CYD 2.8" TFT + Touchscreen).

The firmware features a **Creality K1 OS-inspired dark dashboard**, precise PTC heater control via slow-PWM time-windowing, automatic temperature/humidity tracking via BME280, a full-featured Web UI, Captive Portal Wi-Fi setup, and over-the-air (OTA) firmware flashing.

---

## Key Features

- **Creality K1 UI Style**: Custom dark-themed layout built with `TFT_eSPI` featuring real-time dual-series charts (Temperature & Humidity), status pills, and intuitive touch controls.
- **Safe Soft-Start PTC Control**: Slow-PWM time-windowed duty cycle control capped at **40% peak power** to prevent core hotspotting and eliminate ABS burning hazards.
- **Active-HIGH Triac Driving**: Fail-safe hardware drive logic ensuring the heater remains completely disengaged during boot, crashes, or flashing.
- **Dual Operating Modes**:
  - **Timer Mode**: Dry filament at target temperatures ($35^\circ\text{C} - 66^\circ\text{C}$) for preset or custom durations ($0.5\text{h} - 24\text{h}$).
  - **Auto Humidity Mode**: Continuously monitor chamber humidity and shut off automatically once target % RH is reached.
- **Comprehensive Safety Safeguards**:
  - Emergency thermal cut-off at $\ge 67^\circ\text{C}$.
  - Overheat cool-down logic with temperature hysteresis.
  - Sensor fault detection (failsafe heater shutoff on I2C disconnect or invalid readouts).
  - Fan interlock: Forces 100% blower speed whenever the heater is active.
  - Post-cooling purge: Exhaust fan runs at 100% until the chamber drops below $40^\circ\text{C}$ upon stopping.
- **Web Dashboard**: Modern mobile-friendly web interface with live telemetry, filament profile selection (ABS / PETG / PLA), fan speed adjustments, and manual duty overrides.
- **Wi-Fi Captive Portal**: Built-in AP setup (`SpacePi_Setup` at `192.168.4.1`) for easy Wi-Fi provisioning stored via ESP32 `Preferences` (NVS).
- **Web OTA Updates**: Flash newly compiled `.bin` binaries directly over Wi-Fi without USB cables.

---

## Hardware Pinout & Wiring (CYD 2.8" ESP32-2432S028R)

| Function | ESP32 GPIO / Pin | Connection Details |
| :--- | :--- | :--- |
| **Heater Control (`PIN_HEATER`)** | GPIO 17 (Pad LED Blue) | **Active-HIGH**: Connect to **Pin 1 (Anode)** of EL3063 Optocoupler |
| **Optocoupler Cathode** | GND | Connect **Pin 2 (Cathode)** of EL3063 to **GND** (e.g., AMS1117 Pin 1 GND) |
| **Fan PWM (`PIN_FAN`)** | GPIO 16 (Pad LED Green) | 5kHz PWM signal to Fan MOSFET Gate |
| **I2C SDA** | GPIO 27 | CN1 Header Pin 3 (BME280 SDA) |
| **I2C SCL** | GPIO 22 | CN1 Header Pin 2 (BME280 SCL) |

> **IMPORTANT HARDWARE MODIFICATION**: 
> - **Isolate Pin 1 of EL3063**: Lift Pin 1 (Anode) of the EL3063 off the stock PCB pad (or cut the 5V trace feeding it). Solder GPIO 17 directly to Pin 1.
> - **Ground Pin 2 of EL3063**: Solder Pin 2 (Cathode) directly to common system ground (GND).
> - *Do not use Active-LOW on stock 5V pull-ups as the 3.3V logic level from ESP32 cannot fully switch off the optocoupler diode.*

---

## Required Libraries

Install the following libraries via the Arduino IDE Library Manager or PlatformIO:

- `TFT_eSPI` (configured for ST7789 / ILI9341 on CYD)
- `XPT2046_Touchscreen`
- `Adafruit BME280 Library`
- `Adafruit Unified Sensor`

---

## Getting Started

1. **Configure Display Settings**: Ensure your `TFT_eSPI` `User_Setup.h` matches the pin configuration and display driver of your CYD board.
2. **Compile & Flash**: Open the `.ino` file in Arduino IDE or PlatformIO, select **ESP32 Dev Module**, and upload.
3. **Wi-Fi Provisioning**:
   - On first boot, the screen enters **WIFI SETUP MODE**.
   - Connect to the Wi-Fi AP: `SpacePi_Setup`.
   - Open `http://192.168.4.1` in your browser, select your local Wi-Fi SSID, enter your password, and save.
4. **Access Web Interface**:
   - Connect to the same local network and open `http://spacepi.local` or the assigned IP address.
   - For OTA updates, visit `http://spacepi.local/update`.

---

## License

This project is licensed under the MIT License - feel free to use and modify for personal or commercial projects.

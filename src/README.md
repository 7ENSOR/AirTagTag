# AirTagTag
ESP32 AirTag Detector with Captive Portal -  Apple AirTag detection system using an ESP32. Bluetooth Low Energy (BLE) scans / detects AirTags and other BLE devices, CAPTIVE PORTAL hosted by the ESP32. Dynamic updates, trends based on RSSI values device names and distances.

Key Features:

    Real-Time BLE Scanning: Continuously scans for BLE devices, specifically identifying AirTags.
    AirTag Detection Counter: Maintains a count of each detected AirTag, updating only when the device is within range.
    RSSI Trends: Displays RSSI trends with color coding:
        Red: Signal strength is increasing.
        Green: Signal strength is stable.
        Blue: Signal strength is decreasing.
    Device Distance Estimation: Calculates the approximate distance of the AirTag based on RSSI.
    Captive Portal:
        Lists all detected BLE devices with details.
        Displays AirTag detection counts and trends at the top.
        Accessible through the ESP32's WiFi hotspot.
    Clear and Reset: A button to clear all data and start fresh.
    Automatic Captive Portal: Opens automatically for easy access to device data.

Instructions:

    Hardware Requirements:
        ESP32 development board.
        BLE-enabled AirTags or other BLE devices for testing.
    Setup:
        Flash the provided firmware to your ESP32 using PlatformIO or Arduino IDE.
        Power the ESP32 and connect to the WiFi hotspot ESP32-BLE-Portal.
    Access the Portal:
        Open a browser and navigate to http://192.168.4.1 (default IP address).
    View AirTag Details:
        Check the count, RSSI trends, and other details of detected AirTags.
    Clear Data:
        Use the "Clear" button to reset the detection counts and trends.

Enhancements:

    Dynamic Trend Updates: Tracks RSSI trends for proximity estimation.
    Device Details: Displays additional information like service UUIDs and names.
    Responsive Captive Portal: Dynamically updates device data without restarting.

How It Works:

    BLE Scanning:
        Runs continuously in a separate task.
        Detects devices with Apple manufacturer data (0x4C) to identify AirTags.
    Captive Portal:
        Uses ESPAsyncWebServer to serve data over a local network.
        Displays data dynamically with trends and counts.
    RSSI and Distance:
        RSSI values are used to estimate distance and calculate trends.

Future Improvements:

    Add historical graphs for RSSI trends.
    Enable JSON API for external integrations.
    Optimize memory management for larger device counts.

Dependencies:

    ESPAsyncWebServer
    AsyncTCP
    ESP32 BLE Arduino

Usage Scenarios:

    Track AirTag proximity and presence in a given area.
    Debugging and testing BLE devices.
    Educational projects to explore BLE technology.

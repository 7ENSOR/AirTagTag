#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <unordered_map>
#include <map>

BLEScan* pBLEScan;
AsyncWebServer server(80);

// Store detected AirTags and counts
std::unordered_map<std::string, int> airtagCounts; // To store AirTag addresses and detection counts
std::unordered_map<std::string, int> airtagRSSI;   // To store AirTag RSSI values
std::unordered_map<std::string, String> airtagTrend; // To store AirTag trends (increasing, decreasing, stable)
std::unordered_map<std::string, String> airtagDetails; // To store additional AirTag details
bool isScanning = false; // Flag to indicate scanning state

// Function to estimate distance based on RSSI
float calculateDistance(int rssi) {
  int txPower = -59; // Typical value for 1 meter
  if (rssi == 0) {
    return -1.0; // If we cannot determine distance
  }
  float ratio = rssi * 1.0 / txPower;
  if (ratio < 1.0) {
    return pow(ratio, 10);
  } else {
    return (0.89976) * pow(ratio, 7.7095) + 0.111;
  }
}

// Custom callback to handle detected devices
class CustomAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    std::string address = advertisedDevice.getAddress().toString();
    String details = "";

    // Check for AirTag using manufacturer data
    if (advertisedDevice.haveManufacturerData()) {
      std::string manufacturerData = advertisedDevice.getManufacturerData();
      if (manufacturerData.size() > 2 && manufacturerData[0] == 0x4C && manufacturerData[1] == 0x00) {
        int rssi = advertisedDevice.getRSSI();
        float distance = calculateDistance(rssi);

        // Update trend based on previous RSSI
        if (airtagRSSI.find(address) != airtagRSSI.end()) {
          if (rssi > airtagRSSI[address]) {
            airtagTrend[address] = "red"; // Increasing
          } else if (rssi < airtagRSSI[address]) {
            airtagTrend[address] = "blue"; // Decreasing
          } else {
            airtagTrend[address] = "green"; // Stable
          }
        } else {
          airtagTrend[address] = "green"; // Default to stable for new devices
        }

        // Update RSSI and counts
        airtagRSSI[address] = rssi;
        if (airtagCounts.find(address) == airtagCounts.end()) {
          airtagCounts[address] = 1;
        } else {
          airtagCounts[address]++;
        }

        // Construct details
        details += "RSSI: " + String(rssi) + " | Distance: " + String(distance) + " meters";
        if (advertisedDevice.haveName()) {
          details += " | Name: " + String(advertisedDevice.getName().c_str());
        }
        if (advertisedDevice.haveServiceUUID()) {
          details += " | Service UUID: " + String(advertisedDevice.getServiceUUID().toString().c_str());
        }
        airtagDetails[address] = details;

        // Print AirTag details
        Serial.printf("AirTag Detected - Address: %s | %s | Count: %d | Trend: %s\n", 
                      address.c_str(), details.c_str(), airtagCounts[address], airtagTrend[address].c_str());
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Scanner...");
  BLEDevice::init("ESP32_BLE_Scanner");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new CustomAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Active scan for more data
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(50);

  // Initialize WiFi for Captive Portal
  WiFi.softAP("ESP32-BLE-Portal");
  delay(100);

  // Open captive portal automatically
  Serial.printf("AP IP Address: http://%s\n", WiFi.softAPIP().toString().c_str());

  // Define Captive Portal Content
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><head><title>BLE Devices</title></head><body><h1>BLE Devices</h1>";
    html += "<h2>AirTags Detected:</h2><ul>";
    for (const auto& entry : airtagCounts) {
      String trendColor = airtagTrend[entry.first].c_str();
      html += "<li><span style='color:" + trendColor + "'>";
      html += "Address: " + String(entry.first.c_str()) + " | Count: " + String(entry.second);
      html += " | " + airtagDetails[entry.first];
      html += "</span></li>";
    }
    html += "</ul><button onclick=\"location.href='/clear'\">Clear</button></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/clear", HTTP_GET, [](AsyncWebServerRequest *request) {
    airtagCounts.clear();
    airtagRSSI.clear();
    airtagTrend.clear();
    airtagDetails.clear();
    request->redirect("/");
  });

  server.begin();

  // Start continuous scanning
  isScanning = true;
  xTaskCreatePinnedToCore(
    [] (void* pvParameters) {
      while (true) {
        pBLEScan->start(5, false);
        delay(100);
      }
    },
    "BLEScanTask",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );
}

void loop() {
  // No actions required in loop as scanning is handled in a separate task
}
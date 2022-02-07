#include <BLEDevice.h>
#include <WiFi.h>
#include <Wire.h>
#include <Esp.h>
#include <HTTPClient.h>
#include "Adafruit_MCP9808.h"
#include "driver/timer.h"

#define TIMER_DIVIDER (16)

TaskHandle_t emergencyInterrupt;
TaskHandle_t getVitalSigns;

// Two services unique to the Jumper Oximeter used for the study
static BLEUUID serviceUUID("INSERT BLE SERVICE UUID HERE");
static BLEUUID charUUID("INSERT BLE CHARACTER UUID HERE");

// Necessary variables for the BLE connection
BLEClient* client = BLEDevice::createClient();
static boolean doConnect  = false;
static boolean connected  = false;
static boolean doScan     = false;
static BLERemoteCharacteristic* remoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

struct wifiVars {
  const char* SSID;
  const char* pass;
  const char* serverName;
  int connectingCounter;
  String apiKeyValue;
  String query;
};

wifiVars esp32Wifi = {"INSERT SSID HERE", "INSERT PASSWORD OF SSID HERE", "INSERT LINK TO SERVER HERE", 0, "INSERT API KEY HERE", ""};

// For measuring body temperature
Adafruit_MCP9808 tempSensor = Adafruit_MCP9808();

// For the timer
double elapsedTime;
int number = 0;

timer_config_t config = {
  .alarm_en = TIMER_ALARM_EN,
  .counter_en = TIMER_PAUSE,
  .intr_type =  TIMER_INTR_LEVEL,
  .counter_dir = TIMER_COUNT_UP,
  .auto_reload = TIMER_AUTORELOAD_DIS,
  .divider = TIMER_DIVIDER
};

// For the button interrupt
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

struct Button {
  const int pin;
  bool pressed;
};

Button emerBtn = {4, false};

// Defining function for checking BLE connection
class MyClientCallbacks: public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    connected = true;
    Serial.println("Connected!");
  }

  void onDisconnect(BLEClient* client) {
    connected = false;
    Serial.println("Disconnected!");
  }
};

// A class to be used for scanning BLE device and retrieve the first one that is the same with the service being looked for
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device Found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if the found device contains the details being looked for
    if(advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

void IRAM_ATTR isr(); // Function for the interrupt associated with the emergency button
static void notifyCallbacks(BLERemoteCharacteristic* remoteCharacteristic, uint8_t* data, size_t length, bool isNotify);    // For processing the data coming from the BLE pulse oximeter
void disconnectToOximeter();
bool connectToOximeter();

void setup() {
  // 115200 - Suitable for ESP32
  Serial.begin(115200);
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  tempSensor.begin();
  
  pinMode(emerBtn.pin, INPUT);  //GPIO4 connected to the push button
  attachInterrupt(emerBtn.pin, isr, FALLING);

  WiFi.mode(WIFI_STA);

  xTaskCreatePinnedToCore(
    emergencyInterruptFunc,
    "Emergency Interrupt",
    10000,
    NULL,
    1,
    &emergencyInterrupt,
    0);

  delay(500);

  xTaskCreatePinnedToCore(
    getVitalSignsFunc,
    "Get Vital Signs",
    20000,
    NULL,
    1,
    &getVitalSigns,
    1);

 delay(500);
}

void emergencyInterruptFunc(void *pvParameters) {
  Serial.print("Emergency Interrupt running on core ");
  Serial.println(xPortGetCoreID());
  
  for(;;) {
    if (emerBtn.pressed) {
      // Get the elapsed time from pressing the button until executing this
      emerBtn.pressed = false;
      timer_pause(TIMER_GROUP_0, TIMER_0);
      timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &elapsedTime);
      Serial.print("Elapsed Time for the Emergency Button: ");
      Serial.println(String(elapsedTime) + "seconds.");
    }
    delay(1000);
  }
}

void getVitalSignsFunc(void *pvParameters) {
  Serial.print("Getting vital signs running on core ");
  Serial.println(xPortGetCoreID()); 
  
  for(;;) {
    // Necessary to be here (than in setup()) so the ESP32 can retrieve data from the BLE oximeter even when the oximeter was turned on late
    Serial.println("Starting BLE Client Application using ESP32 . . .");
    BLEDevice::init("");
  
    // Use a scanner and have a callback to know that there has been a detected BLE device
    BLEScan* scanner = BLEDevice::getScan();
    scanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    scanner->setInterval(1349);
    scanner->setWindow(449);
    scanner->setActiveScan(true);
    scanner->start(10, false);

    // Checks whether connecting process should be done
    // Force stopping the process after the checking
    if(doConnect == true) {
      if(connectToOximeter()) {
        Serial.println("Successfully connected to the desired BLE server.");
        delay(1000);
      } else {
        Serial.println("Unsuccessful connection to the desired BLE server.");
      }
    }
    
    // Do this if there has been established connection with a BLE device
    if(connected) {
      String newValue = "Time since boot " + String(millis() / 1000);
      remoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    } else if(doScan) {
      BLEDevice::getScan()->start(0); // Starting the scan after disconnection
    }

    // Disconnect from the BLE pulse oximeter to establish WiFi connection (BLE and WiFi have shared radio in ESP32)
    disconnectToOximeter();
  
    WiFi.begin(esp32Wifi.SSID, esp32Wifi.pass);
    while (WiFi.status() != WL_CONNECTED) {
      if (esp32Wifi.connectingCounter == 10) {
        esp32Wifi.connectingCounter = 0;
        ESP.restart();
        return;
      }
      esp32Wifi.connectingCounter += 1;
      delay(500);
      Serial.println("Connecting to WiFi ...");
    }

    // If successfully connected to the Wi-Fi, reset the connectingCounter
    esp32Wifi.connectingCounter = 0;
  
    // Print ESP32 Local IP Address
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    
    WiFiClient wifiClient;
    HTTPClient http;

    // The domain name with URL path or IP address with path
    http.begin(wifiClient, esp32Wifi.serverName);

    // Specify content-type header
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    float bodyTemp = tempSensor.readTempC();
    String httpRequestData = "api_key=" + esp32Wifi.apiKeyValue + esp32Wifi.query + "&body_temp=" + bodyTemp + "";
    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);
    
    // Send HTTP POST request
    int httpRequestCode = http.POST(httpRequestData);
    delay(1000);
    
    if(httpRequestCode > 0) {
      Serial.print("httpRequestCode: ");
      Serial.println(httpRequestCode);
    } else {
      Serial.print("Error code: ");
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpRequestCode).c_str());
    }
    
    // Free resources
    http.end();
    WiFi.disconnect();
    Serial.println("WiFi Disconnected after being connected.");
    Serial.println();
    delay(1000);
  }
  
}

void loop() {
  
}

void IRAM_ATTR isr() {
  portENTER_CRITICAL(&buttonMux);
  emerBtn.pressed = true;
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
  portEXIT_CRITICAL(&buttonMux);
}

static void notifyCallbacks(BLERemoteCharacteristic* remoteCharacteristic, uint8_t* data, size_t length, bool isNotify) {
  if(data[0] == 129) {
    if(data[1] == 255) {
      connected = false;
      Serial.println("No vital sign values received");
    } else {
      // Serial.println("Vital sign values received.");
      // For checking the retrieved values (Serial monitor)
      Serial.print("Data: ");
      for(int i = 0; i < length; i++) {
        Serial.print(data[i]);
        Serial.print(", "); 
      }
      Serial.println();

      // Retrieved PI level from the oximeter must be divided by 10
      float PILevel = data[3];
      PILevel = PILevel / 10;
      esp32Wifi.query = "&pulse_rate=" + String(data[1]) + "&oxygen_sat=" + String(data[2]) + "&PI_level=" + String(PILevel);
    }
  }
}

// Function for connecting to the BLE pulse oximeter
bool connectToOximeter() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  // BLEClient* client = BLEDevice::createClient();
  // Serial.println("- Created client.");

  client->setClientCallbacks(new MyClientCallbacks());

  // Connecting to the device
  client->connect(myDevice);
  Serial.println("Connected to the BLE device.");

  // Retrieve data from the device
  BLERemoteService* remoteService = client->getService(serviceUUID);
  if(remoteService == nullptr) {
    Serial.print("Failed to find the desired service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    client->disconnect();
    
    return false;
  }
  Serial.println("Recognized and found service UUID.");

  // Get the characteristics from the device
  remoteCharacteristic = remoteService->getCharacteristic(charUUID);
  if(remoteCharacteristic == nullptr) {
    Serial.print("Failed to find the characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    client->disconnect();

    return false;
  }
  Serial.println("Recognized and found characteristic UUID.");

  // Reading the value of the characteristic
  if(remoteCharacteristic->canRead()) {
    std::string value = remoteCharacteristic->readValue();
    Serial.print("The characteristic value is: ");
    Serial.println(value.c_str());
  }

  if(remoteCharacteristic->canNotify()) {
    remoteCharacteristic->registerForNotify(notifyCallbacks);
  }

  return true;
}

void disconnectToOximeter() {
  client->disconnect();
  Serial.println("Disconnected to the BLE device.");
}

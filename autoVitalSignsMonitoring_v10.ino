/**
 *Wearable Vital Signs Monitoring Device (Version 10) 
 *Author: Jan Luis Antoc
 *
 *
 *NOTES:
 *- Changed WiFi SSID and password based on the available local network
 *- Include a variable that stores the status of the emergency button (under the esp32Wifi)
 *- Makerlab 16x2 LCD module address - 0x2A / 0x3F if from e-gizmo
 *
 *TODO:
 * - Make sure that the SOS message does not send again (e.g., hypothermia was sent again after a long while)
 * - Fix issue with sending SOS when emergency button has been pressed. Sometimes, the emergency was not triggered in the general com device of the caregiver.
 * - Fix issue with critical value message. Sometimes, no message in general com device of the caregiver
 * - Bring back message to LCD saying "Connecting to WiFi"
 * - INPUT_PULLDOWN for the push button inputs
 * - Put WiFi connection in setup (never disconnect anymore)
 * - Fix crashing of the system when sendSOS was done during preparation to read temperature
 * - Solve issue with lld_pdu_get_tx_flush_nb HCI packet count mismatch (0, 1)

 *DATE: October 8, 2022
**/

#include <BLEDevice.h>
#include <WiFi.h>
#include <Wire.h>
#include <Esp.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include "esp_sleep.h"
#include "time.h"
//#include "Adafruit_MCP9808.h"
#include "driver/timer.h"

#define TIMER_DIVIDER (16)

// Sleeping time
//#define uS_TO_S_FACTOR  1000000 // Microsecond factor
//#define TIME_TO_SLEEP   0.1     // Change this to 100 if testing so the sleep will not be too long

// Handler of tasks in the system
TaskHandle_t restartInterrupt;
TaskHandle_t emergencyInterrupt;
TaskHandle_t getVitalSigns;

// Two services unique to the Jumper Oximeter used for the study
static BLEUUID serviceUUID("CDEACB80-5235-4C07-8846-93A37EE6B86D");
static BLEUUID charUUID("CDEACB81-5235-4C07-8846-93A37EE6B86D");

// Necessary variables for the BLE connection
BLEClient* client = BLEDevice::createClient();
static boolean doConnect  = false;
static boolean connected  = false;
static boolean doScan     = false;
static BLERemoteCharacteristic* remoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

// Necessary data for connecting to local network and the data to be transmitted
struct wifiVars {
  const char* SSID;
  const char* pass;
  const char* serverName;
  int connectingCounter;
  int emerBtnStatus;
  String apiKeyValue;
  String vitalsID;
  String query;
};

//Domain name of GENCOMM Device
const char* host = "192.168.1.120";
const int httpPort = 80; 

/*****WiFi1 Details*****/
// SSID: PLDTHOMEFIBRdec98
// PASS: PLDTWIFImfk7v
// LINK: http://192.168.1.6/thesis/sendVitalSigns.php

/*****WiFi2 Details*****/
// SSID: Hanging Malamig
// PASS: Z1bIdY=_=9@3
// LINK: http://192.168.0.120/vitalSignsTest/sendVitalSigns.php

/*****WiFi3 Details*****/
// SSID: homespital_wifi
// PASS: tiuresidences
// LINK: http://192.168.1.102/thesis/dev-router/sendVitalSigns.php


// Always make sure that the server address is MODIFIABLE
wifiVars esp32Wifi = {"homespital_wifi", 
                      "#tiuresidences", 
                      "http://192.168.1.102/thesis/dev-router/sendVitalSigns.php", 
                      0, 
                      0, 
                      "y8His941pHq", 
                      "B00001", 
                      ""};

// For measuring body temperature
// Adafruit_MCP9808 tempSensor = Adafruit_MCP9808();
Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();

// For the timer
double elapsedTime;
// int number = 0;

timer_config_t config = {
  .alarm_en = TIMER_ALARM_EN,
  .counter_en = TIMER_PAUSE,
  .intr_type =  TIMER_INTR_LEVEL,
  .counter_dir = TIMER_COUNT_UP,
  .auto_reload = TIMER_AUTORELOAD_DIS,
  .divider = TIMER_DIVIDER
};

// For the button interrupts
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

struct Button {
  const int pin;
  bool pressed;
};

Button restartBtn = {32, false};
Button emerBtn = {33, false};

// For the LCD module
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);

// For tracking the previous and current messages to be shown in the LCD module
// TEST: Aims to solve the flickering of the LCD
String row0PrevMessage = "";
String row0NextMessage = "";
String row1PrevMessage = "";
String row1NextMessage = "";

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

// Function for the interrupt associated with the restart and emergency buttons
void IRAM_ATTR restartBtnISR();
void IRAM_ATTR emerBtnISR();
// Function for sending the emergency message
void sendSOS(int typeOfSOS);
// Function for processing the data coming from the BLE pulse oximeter
static void notifyCallbacks(BLERemoteCharacteristic* remoteCharacteristic, uint8_t* data, size_t length, bool isNotify);
// Function for Disconnecting from and connecting to the oximeter
void disconnectToOximeter();
bool connectToOximeter();

void setup() {
  // 115200 - Suitable for ESP32
  delay(3000);
  // Serial monitor for checking using the laptop
  Serial.begin(115200);

  // Setting up the timer, temperature sensor, and the LCD module
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  tempSensor.begin(0x5A);   // Address 0x5A was verified as the I2C address dedicated for the MLX9614 IR temperature sensor

  // Initializing the LCD module
  lcd.init();
  lcd.backlight();
  //lcd.blink();
  lcd.clear();

  // Setting up ten-minute interval in measuring temperature
  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //GPIO4 connected to the push button
  pinMode(emerBtn.pin, INPUT_PULLDOWN);
  pinMode(restartBtn.pin, INPUT_PULLDOWN);
  attachInterrupt(restartBtn.pin, restartBtnISR, FALLING);
  attachInterrupt(emerBtn.pin, emerBtnISR, FALLING);

  WiFi.mode(WIFI_STA);

  // Setting up six seconds for restarting the esp32
  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  xTaskCreatePinnedToCore(
    restartInterruptFunc,
    "Restart Interrupt",
    10000,
    NULL,
    1,
    &restartInterrupt,
    0);
    
  xTaskCreatePinnedToCore(
    emergencyInterruptFunc,
    "Emergency Interrupt",
    10000,
    NULL,
    2,
    &emergencyInterrupt,
    0);

  delay(500);

  xTaskCreatePinnedToCore(
    getVitalSignsFunc,
    "Get Vital Signs",
    20000,
    NULL,
    3,
    &getVitalSigns,
    0);

 delay(500);
}

void restartInterruptFunc(void *pvParameters) {
  Serial.print("Restart Interrupt running on core ");
  Serial.println(xPortGetCoreID());

  // Testing suspension of tasks
  //vTaskSuspend(emergencyInterrupt);
  //vTaskSuspend(getVitalSigns);
  
  for(;;) {
    if (restartBtn.pressed) {
      // Get the elapsed time from pressing the button until executing this
      Serial.println("Restarting ESP32...");

      // Setting the text to be displayed in the LCD when restart button has been pressed
      row0NextMessage = "Restarting. . .";

      if(row0NextMessage != row0PrevMessage) {
        row0PrevMessage = row0NextMessage;
        lcd.clear();
      }
      // lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      //lcd.noDisplay();
      delay(100);
      ESP.restart();
      return;
      // esp_restart();
      //esp_deep_sleep_start();
      //lcd.clear();
    }
    delay(100);
  }
}

void emergencyInterruptFunc(void *pvParameters) {
  Serial.print("Emergency Interrupt running on core ");
  Serial.println(xPortGetCoreID());
  
  for(;;) {
    if (emerBtn.pressed) {
      // Get the elapsed time from pressing the button until executing this
      esp32Wifi.emerBtnStatus = 1;
      emerBtn.pressed = false;

      // Set the text to be displayed in the LCD module
      row0NextMessage = "Sending SOS";
      if(row0NextMessage != row0PrevMessage) {
        row0PrevMessage = row0NextMessage;
        lcd.clear();
      }
      //lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      delay(100);
      sendSOS(0);
      
      timer_pause(TIMER_GROUP_0, TIMER_0);
      timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &elapsedTime);
      Serial.print("Elapsed Time for the Emergency Button: ");
      Serial.println(String(elapsedTime) + "seconds.");
    }
    delay(500);
  }
}


// TO-DO: If the ESP32 dev board could be restarted without always checking restartBtn (due to LCD display issues), try!
void getVitalSignsFunc(void *pvParameters) {
  Serial.print("Getting vital signs running on core ");
  Serial.println(xPortGetCoreID()); 
  //lcd.clear();

  // Read the body temperature after 10 seconds via contactless temperature sensor
  int tempCounter = 5;

  row0NextMessage = "Place your wrist";
  row1NextMessage = "near the sensor";

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
      row0PrevMessage = row0NextMessage;
      row1PrevMessage = row1NextMessage;
      lcd.clear();
  }
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  delay(1500);
  
  while(tempCounter != 0) {
    row0NextMessage = "Reading temp";
    row1NextMessage = "in " + String(tempCounter);

    // Exception here. Requires two ifs because the first row might still the same but the second row is not (countdown)
    if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
      row0PrevMessage = row0NextMessage;
      row1PrevMessage = row1NextMessage;
      lcd.clear();
    }

    //Serial.println(row1NextMessage);
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(row0NextMessage);
    lcd.setCursor(0,1);
    lcd.print(row1NextMessage);
    //lcd.print("in "); lcd.print(tempCounter);
    delay(1000);
    tempCounter--;
  }

  float bodyTemp = tempSensor.readObjectTempC();
  // Add the offset of wrist temperature compared to average ear temperature
  bodyTemp = bodyTemp + 3.0;
  
  // Display the body temperature
  row0NextMessage = "Body Temp:" + String(bodyTemp * 1.0) + "C";

  if(row0NextMessage != row0PrevMessage) {
    row0PrevMessage = row0NextMessage;
    lcd.clear();
  }
  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  //lcd.print("Body Temp:"); lcd.print(bodyTemp, 1); lcd.print("C");
  delay(2000);

  // Connecting to the pulse oximeter
  Serial.println("Starting BLE Client Application using ESP32 . . .");

  row0NextMessage = "Connecting to";
  row1NextMessage = "Pulse Oximeter";

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }
  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  BLEDevice::init("");

  // Use a scanner and have a callback to know that there has been a detected BLE device
  BLEScan* scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scanner->setInterval(1349);
  scanner->setWindow(449);
  scanner->setActiveScan(true);
  scanner->start(10, false);

  if(doConnect == true) {
    Serial.print("doConnect:");
    Serial.println(doConnect);
    if(connectToOximeter()) {
      Serial.println("Successfully connected to the desired BLE server.");

      row0NextMessage = "Successful";
      row1NextMessage = "Connection!";

      if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
        row0PrevMessage = row0NextMessage;
        row1PrevMessage = row1NextMessage;
        lcd.clear();
      }
      //lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      lcd.setCursor(0,1);
      lcd.print(row1NextMessage);
      delay(1000);
      }
    } else {
      Serial.println("Unsuccessful connection to the desired BLE server.");

      row0NextMessage = "Unsuccessful";
      row1NextMessage = "Connection!";

      if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
        row0PrevMessage = row0NextMessage;
        row1PrevMessage = row1NextMessage;
        lcd.clear();
      }
      
      //lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      lcd.setCursor(0,1);
      lcd.print(row1NextMessage);
      delay(1000);
    }

    // Disconnect from the BLE pulse oximeter to establish WiFi connection (BLE and WiFi have shared radio in ESP32)
    if(connected) {
      disconnectToOximeter();
    }

    WiFi.begin(esp32Wifi.SSID, esp32Wifi.pass);
    while (WiFi.status() != WL_CONNECTED) {
      if (esp32Wifi.connectingCounter == 500) {
        esp32Wifi.connectingCounter = 0;
        ESP.restart();
        return;
      } else {
        esp32Wifi.connectingCounter += 1;
        // Optimize this so the screen would not always blink due to clearing
        Serial.println("Connecting to WiFi ...");
        //lcd.clear();
        //lcd.setCursor(0,0);
        //lcd.print("Connecting");
        //lcd.setCursor(0,1);
        //lcd.print("to WiFi");
        delay(500);
      }
   }

  // If successfully connected to the Wi-Fi, reset the connectingCounter
  esp32Wifi.connectingCounter = 0;

  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Connected");
  //lcd.setCursor(0,1);
  //lcd.print(WiFi.SSID());
  //delay(1000);

  // Sending vital signs data via WiFi
  WiFiClient wifiClient;
  HTTPClient http;

  // The domain name with URL path or IP address with path
  http.begin(wifiClient, esp32Wifi.serverName);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String httpRequestData = "api_key=" + esp32Wifi.apiKeyValue + esp32Wifi.query + "&BodyTemp=" + bodyTemp + "&vitals_id=" + esp32Wifi.vitalsID + "";
  Serial.print("httpRequestData: ");
  Serial.println(httpRequestData);

  // Send HTTP POST request
  int httpRequestCode = http.POST(httpRequestData);
  delay(1000);

  // Check whether the data were sent from the ESP32 to the local server
  if(httpRequestCode > 0) {
    Serial.print("httpRequestCode: ");
    Serial.println(httpRequestCode);

    row0NextMessage = "Vital signs";
    row1NextMessage = "sent";

    if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
      row0PrevMessage = row0NextMessage;
      row1PrevMessage = row1NextMessage;
      lcd.clear();
    }
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(row0NextMessage);
    lcd.setCursor(0,1);
    lcd.print(row1NextMessage);
    delay(1000);
  } else {
    Serial.print("Error code: ");
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpRequestCode).c_str());

    row0NextMessage = "No vital signs";
    row1NextMessage = "sent";

    if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
      row0PrevMessage = row0NextMessage;
      row1PrevMessage = row1NextMessage;
      lcd.clear();
    }

    delay(1000);
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(row0NextMessage);
    lcd.setCursor(0,1);
    lcd.print(row1NextMessage);
    delay(1000);
    //lcd.clear();
    //lcd.setCursor(0,0);
    //lcd.print("Disconnected");
    //lcd.setCursor(0,1);
    //lcd.print(WiFi.SSID());
    //delay(1000); 
  }

  // Free resources
  http.end();

  // reset the emergency button status to 0
  esp32Wifi.emerBtnStatus = 0;

  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Disconnected");
  //lcd.setCursor(0,1);
  //lcd.print(WiFi.SSID());
  //delay(1000);
  
  Serial.println();
  WiFi.disconnect();
  Serial.println("WiFi Disconnected after being connected.");

  // Emergency status due to abnormal body temperature
  if (bodyTemp > 37.2) {
    sendSOS(1);
  } else if (bodyTemp < 36.1) {
    sendSOS(2);
  } else {
    // Do nothing
  }

  row0NextMessage = "Device on";
  row1NextMessage = "Standby";
    
  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
      row0PrevMessage = row0NextMessage;
      row1PrevMessage = row1NextMessage;
      lcd.clear();
  }

  // Show device is on standby
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  
  // Stop getting vital signs unless the device has been restarted
  vTaskSuspend(getVitalSigns);
}

void loop() {
  // No code here
}

void IRAM_ATTR restartBtnISR() {
  // Suspending the tasks here worked rather than doing these in the interrupts function
  vTaskSuspend(emergencyInterrupt);
  vTaskSuspend(getVitalSigns);
  
  portENTER_CRITICAL(&buttonMux);
  restartBtn.pressed = true;
  portEXIT_CRITICAL(&buttonMux);
}

void IRAM_ATTR emerBtnISR() {
  // Variables that will track the timing of the emergency button interrupts
  // https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
  // Not yet resolved. Solve bouncing problem
  unsigned long button_time = 0;
  unsigned long last_button_time = 0;
  vTaskSuspend(getVitalSigns);
  
  button_time = millis();
  if (button_time - last_button_time > 1000) {
    last_button_time = button_time;
    portENTER_CRITICAL(&buttonMux);
    emerBtn.pressed = true;
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
    portEXIT_CRITICAL(&buttonMux);
  }
}

// Function for sending emergency notification to the caregiver
void sendSOS(int typeOfSOS) {
  // There should be an input to sendSOS
  // 0 - emergency button
  // 1 - high temperature
  // 2 - low temperature
  // 3 - low oxygen saturation
  // 4 - high pulse rate
  // 5 - low pulse rate
  
  WiFi.begin(esp32Wifi.SSID, esp32Wifi.pass);
  while (WiFi.status() != WL_CONNECTED) {
    if (esp32Wifi.connectingCounter == 500) {
      esp32Wifi.connectingCounter = 0;
      ESP.restart();
      return;
    } else {
      esp32Wifi.connectingCounter += 1;
      Serial.println("Connecting to WiFi ...");
      //lcd.clear();
      //lcd.setCursor(0,0);
      //lcd.print("Connecting");
      //lcd.setCursor(0,1);
      //lcd.print("to WiFi");
      delay(100);
    }
  }

  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  // Showing connected already to WiFi
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Connected");
  //lcd.setCursor(0,1);
  //lcd.print(WiFi.SSID());
  delay(100);

  // Initializing WiFiClient and HTTPClient for sending SOS message
  WiFiClient wifiClient;
  HTTPClient httpClient;

  lcd.clear();
  while(!wifiClient.connect(host, httpPort)) {
    //if(typeOfSOS == 0) {
      //Serial.println("Gen Com Device connection failed");

      //row0NextMessage = "SOS NOT SENT";

      //if(row0PrevMessage != row0NextMessage) {
        //row0PrevMessage = row0NextMessage;
        //lcd.clear();
      //}
      //lcd.setCursor(0,0);
      //lcd.print(row0NextMessage);
      delay(100); 
    }
  }

  // Sending the SOS message
  String url;

  switch(typeOfSOS) {
    case 0:
      url = "/?state=2&lcd=EMERGENCY I NEED IMMEDIATE HELP";
      break;
    case 1:
      url = "/?state=2&lcd=FEVER!";
      break;
    case 2:
      url = "/?state=2&lcd=LOW BODY        TEMPERATURE!";
      break;
    case 3:
      url = "/?state=2&lcd=FAST PULSE      RATE!";
      break;
    case 4:
      url = "/?state=2&lcd=WEAK PULSE      RATE!";
      break;
    case 5:
      url = "/?state=2&lcd=LOW OXYGEN      SATURATION!";
      break;
    default:
      url = "/?state=2&lcd=EMERGENCY I NEED IMMEDIATE HELP";
  }
  
  // String url = "/?state=2&lcd=SOS: Need help!";
  Serial.println(url);
  wifiClient.print("GET " + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

  while (wifiClient.available()){
    if (wifiClient.available()){
      String line = wifiClient.readStringUntil('\n');
      Serial.println(line);
      }
  }

  lcd.clear();
  lcd.setCursor(0,0);
  if(typeOfSOS == 0) {
    row0NextMessage = "SOS Sent";

    if(row0PrevMessage != row0NextMessage) {
      row0PrevMessage = row0NextMessage;
      lcd.clear();
    }
    lcd.setCursor(0,0);
    lcd.print("SOS Sent");
  }
  
  delay(100);
  esp32Wifi.connectingCounter = 0;
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

      // Testing whether there should be sending of SOS due to abnormal vital signs
      // data[2] = 92;
      if(data[1] > 100) {
        sendSOS(3);
      } else if (data[1] < 60) {
        sendSOS(4);
      } else if (data[2] < 95) {
        sendSOS(5);
      }
      
      esp32Wifi.query = "&PulseRate=" + String(data[1]) + "&OxygenSat=" + String(data[2]) + "&PerfusionIndex=" + String(PILevel);
      Serial.println(esp32Wifi.query);
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
  Serial.println("Disconnected from the BLE device.");
}

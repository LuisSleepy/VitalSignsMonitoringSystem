/**
 * Automated Vital Signs Retrieval System - version 1
 * Author: Jan Luis Antoc
 * 
 * NOTES:
 * - Working so far. But need to make sure that getting vital signs will not work during emergency and restart.
 * 
 * TO-DO:
 * - Debug exception reason: Stack canary watchpoint triggered (BTU_TASK)
 *    - Worked using 30000 as the stack size but sometimes the stack canary still triggers. Not working if 50000.
 * - Solve issue with panic core 0. Might be due to stack size. Also try to include longer delays in preparation
 * 
 * 
 * DATE: November 10, 2022
 */

#include <BLEDevice.h>              // For connecting the ESP32 to the BLE-powered pulse oximeter
#include <WiFi.h>                   // For connecting the ESP32 to the local network                  
#include <Esp.h>                    // For the restarting of ESP32 (not sure, test if this is really needed)
#include <WiFiClient.h>             // For the Wi-Fi communication with the general communication buzzer of the caregiver
#include <HTTPClient.h>             // For sending vital signs and the message to the general communication buzzer
#include <LiquidCrystal_I2C.h>      // For the display
#include <Adafruit_MLX90614.h>      // For the IR temperature sensor
#include "driver/timer.h"           // For the timer


#define TIMER_DIVIDER (16)


// Handler of tasks in the system (restart, emergency, vitals)
TaskHandle_t restartInterrupt;
TaskHandle_t emergencyInterrupt;
TaskHandle_t getVitalSigns;


// Two services unique to the Jumper Oximeter used for the study
static BLEUUID serviceUUID("CDEACB80-5235-4C07-8846-93A37EE6B86D");
static BLEUUID charUUID("CDEACB81-5235-4C07-8846-93A37EE6B86D");

// Necessary variables for the BLE connection
BLEClient* BLEDeviceClient = BLEDevice::createClient();
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
  String apiKeyValue;
  String vitalsID;
  String query;
};

// Setup for the static IP address of the device
IPAddress local_IP(192,168,1,180);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,0,0);

//Domain name of GENCOMM Device
const char* host = "192.168.1.120";
const int httpPort = 80; 

// UPDATE this based on the local Wi-Fi credentials and the local web application
wifiVars esp32Wifi = {"homespital_wifi", 
                      "#tiuresidences", 
                      "http://192.168.1.2/vitalSignsTest/sendVitalSigns.php", 
                      0,
                      "y8His941pHq", 
                      "B00001", 
                      ""};


// Defining the temperature sensor
Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();
double objectEmissivity = 0.98;


// For the timers
double elapsedTime;
double inBetweenElapsedTime;

timer_config_t timerConfig = {
  .alarm_en = TIMER_ALARM_EN,
  .counter_en = TIMER_PAUSE,
  .intr_type =  TIMER_INTR_LEVEL,
  .counter_dir = TIMER_COUNT_UP,
  .auto_reload = TIMER_AUTORELOAD_DIS,
  .divider = TIMER_DIVIDER
};


// For the button interrupts (Explore this if it is needed to have two of these, one for restart and one for emergency
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;


struct Button {
  const int pin;
  bool pressed;
};
// The two push buttons
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
  void onConnect(BLEClient* BLEDeviceClient) {
    connected = true;
    Serial.println("Connected!");
  }

  void onDisconnect(BLEClient* BLEDeviceClient) {
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
void sendSOS(int typeOfSOS);


// Function for processing the data coming from the BLE pulse oximeter
static void notifyCallbacks(BLERemoteCharacteristic* remoteCharacteristic, uint8_t* data, size_t length, bool isNotify);
// Function for Disconnecting from and connecting to the oximeter
void disconnectToOximeter();
bool connectToOximeter();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  timer_init(TIMER_GROUP_0, TIMER_0, &timerConfig);   // This timer would be for measuring the delay between pressing the emergency button and successful triggering the buzzer
  timer_init(TIMER_GROUP_0, TIMER_1, &timerConfig);   // This timer is to prevent the issue with bouncing emergency button
  timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
  timer_start(TIMER_GROUP_0, TIMER_1);
  
  tempSensor.begin(0x5A);                             // Address 0x5A was verified as the I2C address dedicated for the MLX9614 IR temperature sensor
  
  // Initializing the LCD module
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Declaring the pins connected to the push buttons
  pinMode(emerBtn.pin, INPUT_PULLDOWN);
  pinMode(restartBtn.pin, INPUT_PULLDOWN);
  attachInterrupt(restartBtn.pin, restartBtnISR, FALLING);
  attachInterrupt(emerBtn.pin, emerBtnISR, FALLING);

  WiFi.mode(WIFI_STA);
  // Set up static IP address
  if(!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA failed to configure!");
  } else {
    Serial.println("STA configuration successful!");
  }

  xTaskCreatePinnedToCore(
    restartInterruptFunc,
    "Restart Interrupt",
    20000,
    NULL,
    1,
    &restartInterrupt,
    0);

  delay(500);

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
    35000,
    NULL,
    1,
    &getVitalSigns,
    1);

 delay(500);
}



/*
 * Function for what to do when the restart button has been pressed
 */
void restartInterruptFunc(void *pvParameters) {
  Serial.print("Restart Interrupt running on core ");
  Serial.println(xPortGetCoreID());
  
  for(;;) {
    if (restartBtn.pressed) {
      vTaskSuspend(getVitalSigns);
      vTaskSuspend(emergencyInterrupt);
      
      Serial.println("Restarting ESP32...");
      restartBtn.pressed = false;
      // Setting the text to be displayed in the LCD when restart button has been pressed
      lcd.clear();
      row0NextMessage = "Restarting. . .";

      if(row0NextMessage != row0PrevMessage) {
        row0PrevMessage = row0NextMessage;
        lcd.clear();
      }
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      delay(1000);
      ESP.restart();
    }
    delay(1000);
  }
}


/*
 * Function for what to do when the emergency button has been pressed
 */
void emergencyInterruptFunc(void *pvParameters) {
  Serial.print("Emergency Interrupt running on core ");
  Serial.println(xPortGetCoreID());
  
  for(;;) {
    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_1, &inBetweenElapsedTime);
    if (emerBtn.pressed && inBetweenElapsedTime >= 1) {
      vTaskSuspend(getVitalSigns);
      emerBtn.pressed = false;

      // Set the text to be displayed in the LCD module
      row0NextMessage = "Sending SOS";
      if(row0NextMessage != row0PrevMessage) {
        row0PrevMessage = row0NextMessage;
        lcd.clear();
      }
      
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      sendSOS(0);

      // Get the time consumed in successful or unsuccessful sending of emergency message
      timer_pause(TIMER_GROUP_0, TIMER_0);
      timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &elapsedTime);
      Serial.print("Elapsed Time for the Emergency Button: ");
      Serial.println(String(elapsedTime) + "seconds.");

      // Count again the time between possible bouncing of emergency button
      timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
      timer_start(TIMER_GROUP_0, TIMER_1);
    }
    delay(1000);
  }
}



/*
 * Function for retrieval of vital signs
 */
void getVitalSignsFunc(void *pvParameters) {
  Serial.print("Getting vital signs running on core ");
  Serial.println(xPortGetCoreID()); 

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
  delay(2000);
  
  while(tempCounter != 0) {
    row0NextMessage = "Reading body";
    row1NextMessage = "temp in " + String(tempCounter);

    if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
      row0PrevMessage = row0NextMessage;
      row1PrevMessage = row1NextMessage;
      lcd.clear();
    }

    lcd.setCursor(0,0);
    lcd.print(row0NextMessage);
    lcd.setCursor(0,1);
    lcd.print(row1NextMessage);
    delay(1000);
    tempCounter--;
  }

  //tempSensor.writeEmissivity(objectEmissivity);
  Serial.print("Emissivity:"); Serial.println(tempSensor.readEmissivity());
  
  float ambientTemp = tempSensor.readAmbientTempC();
  float bodyTemp = tempSensor.readObjectTempC();
  
  // Add the offset of wrist temperature compared to average ear temperature
  if(ambientTemp <= 30) {
    bodyTemp = bodyTemp + 5;
  } else if(ambientTemp > 30 && ambientTemp <=31) {
    bodyTemp = bodyTemp + 4;
  } else if(ambientTemp > 31) {
    bodyTemp = bodyTemp + 2.2;
  }

  Serial.print("Body Temperature: "); Serial.println(bodyTemp);
  Serial.print("Ambient Temperature: "); Serial.println(ambientTemp);
  
  // Display the body temperature
  row0NextMessage = "Body Temp:" + String(bodyTemp * 1.0) + "C";

  if(row0NextMessage != row0PrevMessage) {
    row0PrevMessage = row0NextMessage;
    lcd.clear();
  }
  
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  delay(2000);

  // Connecting to the pulse oximeter
  WiFi.disconnect();
  Serial.println("Starting BLE Client Application using ESP32 . . .");

  int BLEConnectionInd = 0;
  row0NextMessage = "Connecting to";
  row1NextMessage = "Pulse Oximeter";

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }

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

      BLEConnectionInd = 1;
      row0NextMessage = "Successful";
      row1NextMessage = "Connection!";

      if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
        row0PrevMessage = row0NextMessage;
        row1PrevMessage = row1NextMessage;
        lcd.clear();
      }
      
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      lcd.setCursor(0,1);
      lcd.print(row1NextMessage);
      }
    } else {
      Serial.println("Unsuccessful connection to the desired BLE server.");

      BLEConnectionInd = 0;
      row0NextMessage = "Unsuccessful";
      row1NextMessage = "Connection!";

      if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
        row0PrevMessage = row0NextMessage;
        row1PrevMessage = row1NextMessage;
        lcd.clear();
      }
      
      
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      lcd.setCursor(0,1);
      lcd.print(row1NextMessage);
    }
    delay(1000);

    // Disconnect from the BLE pulse oximeter to establish WiFi connection (BLE and WiFi have shared radio in ESP32)
    if(connected) {
      disconnectToOximeter();
    }

    WiFi.begin(esp32Wifi.SSID, esp32Wifi.pass);
    while (WiFi.status() != WL_CONNECTED) {
      if (esp32Wifi.connectingCounter == 2500) {
        esp32Wifi.connectingCounter = 0;
        ESP.restart();
      } else {
        esp32Wifi.connectingCounter += 1;
        Serial.println("Connecting to WiFi ...");

        row0NextMessage = "Connecting to";
        row1NextMessage = "Wi-Fi";
        
        if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
          row0PrevMessage = row0NextMessage;
          row1PrevMessage = row1NextMessage;
          lcd.clear();
        }
        
        lcd.setCursor(0,0);
        lcd.print(row0NextMessage);
        lcd.setCursor(0,1);
        lcd.print(row1NextMessage);
        delay(100);
      }
   }

  // If successfully connected to the Wi-Fi, reset the connectingCounter
  esp32Wifi.connectingCounter = 0;

  // Show information of successful connection to a local network
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  row0NextMessage = "Connected to";
  row1NextMessage = WiFi.SSID();
        
  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }
  
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  delay(1000);

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
    
    lcd.setCursor(0,0);
    lcd.print(row0NextMessage);
    lcd.setCursor(0,1);
    lcd.print(row1NextMessage);
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

    lcd.setCursor(0,0);
    lcd.print(row0NextMessage);
    lcd.setCursor(0,1);
    lcd.print(row1NextMessage);
  }
  delay(1000);

  // Free resources
  http.end();
  
  Serial.println();

  // Emergency status due to abnormal body temperature
  // NOTE: Make sure that if there is no vital sign retrieved from the pulse oximeter, don't bother to send SOS due to body temperature
  if (bodyTemp > 38.5 && BLEConnectionInd == 1) {
    sendSOS(1);
  } else if (bodyTemp < 34.0 && BLEConnectionInd == 1) {
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
      
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  delay(1000);
  
  // Stop getting vital signs unless the device has been restarted
  vTaskSuspend(getVitalSigns);
}



void loop() {
  // No line of codes here since the program runs through interrupts and prioritization of the tasks assigned to the core 0

}



/**
 * Function related to the interrupt of the restart button
 */
void IRAM_ATTR restartBtnISR() {
  // Suspending the tasks here worked rather than doing these in the interrupts function
  // vTaskSuspend(emergencyInterrupt);
  // vTaskSuspend(getVitalSigns);
  
  portENTER_CRITICAL(&buttonMux);
  restartBtn.pressed = true;
  portEXIT_CRITICAL(&buttonMux);
}



/**
 * Function related to the interrupt of the emergency button
 */
void IRAM_ATTR emerBtnISR() {
  // vTaskSuspend(getVitalSigns);
  
  // Start doing the emergency message
  portENTER_CRITICAL(&buttonMux);
  emerBtn.pressed = true;
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
  portEXIT_CRITICAL(&buttonMux);
}



/**
 * Function for sending emergency notification to the caregiver
 */
void sendSOS(int typeOfSOS) {
  // There should be an input to sendSOS
  // 0 - emergency button
  // 1 - high temperature
  // 2 - low temperature
  // 3 - low oxygen saturation
  // 4 - fast pulse rate
  // 5 - weak pulse rate
  
  WiFi.begin(esp32Wifi.SSID, esp32Wifi.pass);
  while (WiFi.status() != WL_CONNECTED) {
    if (esp32Wifi.connectingCounter == 2500) {
      esp32Wifi.connectingCounter = 0;
      ESP.restart();
    } else {
      esp32Wifi.connectingCounter += 1;
      
      row0NextMessage = "Connecting to";
      row1NextMessage = "Wi-Fi";

      if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
        row0PrevMessage = row0NextMessage;
        row1PrevMessage = row1NextMessage;
        lcd.clear();
      }
      
      lcd.setCursor(0,0);
      lcd.print(row0NextMessage);
      lcd.setCursor(0,1);
      lcd.print(row1NextMessage);
      delay(100);
    }
  }

  esp32Wifi.connectingCounter = 0;
  row0NextMessage = "Connected to";
  row1NextMessage = WiFi.SSID();

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }

  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  delay(300);

  // Initializing WiFiClient and HTTPClient for sending SOS message
  WiFiClient wifiClient;
  HTTPClient httpClient;
  String line = "";
  String url = "";

  row0NextMessage = "Connecting to";
  row1NextMessage = "Caregiver";

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }

  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  delay(300);

  if(wifiClient.connect(host, httpPort)) {
    // Sending the SOS message

    // Determine what type of SOS message must be sent
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

    //Serial.println(url);
    wifiClient.print("GET " + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");
    //delay(300);
    
    //Serial.print("Response: ");
    while((wifiClient.connected() || wifiClient.available()) && line.length() == 0) {
      if(wifiClient.available()) {
        line = wifiClient.readStringUntil('\n');
        Serial.println(line);
      }
    }
  }

  int emergencyResponseLen = line.length();
  Serial.print("Length of response: "); Serial.println(emergencyResponseLen);
  // Inform the patient that SOS has been sent
  // Guaranteed OK if it passed the above while loop

  if(typeOfSOS == 0 && emergencyResponseLen > 0) {
    row0NextMessage = "SOS SENT"; 
  } else if(typeOfSOS == 0 && emergencyResponseLen == 0) {
    row0NextMessage = "SOS NOT SENT";
  }

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }

  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  delay(3000);

  row0NextMessage = "Device on";
  row1NextMessage = "Standby";

  if((row0NextMessage != row0PrevMessage) || (row1NextMessage != row1PrevMessage)) {
    row0PrevMessage = row0NextMessage;
    row1PrevMessage = row1NextMessage;
    lcd.clear();
  }
      
  lcd.setCursor(0,0);
  lcd.print(row0NextMessage);
  lcd.setCursor(0,1);
  lcd.print(row1NextMessage);
  delay(1000);
}



/**
 * Function for getting the vital signs from the pulse oximeter
 */
static void notifyCallbacks(BLERemoteCharacteristic* remoteCharacteristic, uint8_t* data, size_t length, bool isNotify) {
  if(data[0] == 129) {
    if(data[1] == 255) {
      connected = false;
      Serial.println("No vital sign values received");
    } else {
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
      if(data[1] > 105) {
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



/**
 * Function for connecting to the BLE pulse oximeter
 */
bool connectToOximeter() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  // BLEClient* client = BLEDevice::createClient();
  // Serial.println("- Created client.");

  BLEDeviceClient->setClientCallbacks(new MyClientCallbacks());

  // Connecting to the device
  BLEDeviceClient->connect(myDevice);
  Serial.println("Connected to the BLE device.");

  // Retrieve data from the device
  BLERemoteService* remoteService = BLEDeviceClient->getService(serviceUUID);
  if(remoteService == nullptr) {
    Serial.print("Failed to find the desired service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    BLEDeviceClient->disconnect();
    
    return false;
  }
  Serial.println("Recognized and found service UUID.");

  // Get the characteristics from the device
  remoteCharacteristic = remoteService->getCharacteristic(charUUID);
  if(remoteCharacteristic == nullptr) {
    Serial.print("Failed to find the characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    BLEDeviceClient->disconnect();

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



/**
 * Function for disconnecting from the BLE pulse oximeter
 */
void disconnectToOximeter() {
  BLEDeviceClient->disconnect();
  Serial.println("Disconnected from the BLE device.");
}

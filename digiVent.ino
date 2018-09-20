// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
#include <ESP8266WiFi.h>
//#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>  // https://github.com/marvinroger/async-mqtt-client (requires: https://github.com/me-no-dev/ESPAsyncTCP)
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "config.h"           // rename sample_config.h and edit any values needed
#include <BME280I2C.h>        // https://github.com/finitespace/BME280
#include <EnvironmentCalculations.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Input Pins                                                    //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
#define sclPin      D1
#define sdaPin      D2
#define pirPin      D5
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Output Pins                                                   //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
#define intLED1Pin  D4  // D4 Wemos Mini D1 and NodeMCU
#define intLED2Pin  D0  // NodeMCU has a second LED on D0
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Verb/State Conversions                                        //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
#define LEDon       LOW
#define LEDoff      HIGH
#define pirOnStr    "Yes"
#define pirOffStr   "No"
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Poll Times / Misc                                             //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
#define bmePoll        30  // 60 seconds is recommended (sensor could heat up if polled sooner)
#define rePushPoll     300 // push all sensor data every xx seconds - helpful if retain isn't used as this sketch only pushes data if a sensor changes.
#define mqttQOSLevel   2
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// VARS Begin                                                    //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
char mcuHostName[64]; 
char pirTopic[96];    
char lwtTopic[96];  
char bmeTempTopic[96];
char bmeHumiTopic[96];
char bmePresTopic[96];
char bmeFeelTopic[96];
char buildTopic[96];
char rssiTopic[96];
char sBMEfeelValue[5]; 
char sBMEhumValue[5]; 
char sBMEpresValue[7]; 
char sBMEtempValue[5]; 
int pirValue;
int pirStatus;

float BMEdiffTemp = 0.5;  // difference in temperature to trigger a MQTT publish
float BMEdiffHum  = 0.5;
float BMEdiffPres = 0.002;
float BMEdiffFeel = 0.5;
// -+-+
float BMEtempValue;
float BMEnewTempValue;
float BMEhumValue;
float BMEnewHumValue;
float BMEpresValue;
float BMEnewPresValue;
float BMEfeelValue;
float BMEnewFeelValue;
// -+-+
bool bme280Avail = false;
bool initBoot = true;

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);
BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
BME280::PresUnit presUnit(BME280::PresUnit_inHg);
EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Fahrenheit;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
#ifdef DEBUGTELNET
  WiFiServer telnetServer(23);
  WiFiClient telnetClient;
#endif

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Ticker bme280Tick;
Ticker rePushTick;
Ticker led1FlipTick;
Ticker wifiReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// connectToWifi                                                 //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void connectToWifi() {
  debugLn(F("WIFI: Attempting to Connect."));
  WiFi.mode(WIFI_STA);
  WiFi.hostname(mcuHostName);
  WiFi.begin(wifiSSID, wifiPass);
  delay(10);  
  // toggle on board LED as WiFi comes up
  digitalWrite(intLED1Pin, LEDoff);
  while (WiFi.status() != WL_CONNECTED) {
     digitalWrite(intLED1Pin, !(digitalRead(intLED1Pin)));  //Invert Current State of LED  
     delay(70);
     digitalWrite(intLED1Pin, !(digitalRead(intLED1Pin)));   
     delay(45);
  }
  digitalWrite(intLED1Pin, LEDoff);
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// onWifiConnect                                                 //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  long rssi = WiFi.RSSI();
  debugLn(String(F("WIFI: Connected - IP: ")) + WiFi.localIP().toString() + " - RSSI: " + String(rssi) );
  connectToMqtt();
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// onWifiDisconnect                                              //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  debugLn(F("WIFI: Disconnected."));
  mqttReconnectTimer.detach(); // don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// connectToMqtt                                                 //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void connectToMqtt() {
  debugLn(String(F("MQTT: Attempting connection to ")) + String(mqttHost) + " as " + mcuHostName);
  mqttClient.connect();
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Check Sensor                                                  //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void checkBME280()
{
  debugLn(F("BME280: Read"));
  if (bme280Avail) {   
    bme.read(BMEnewPresValue, BMEnewTempValue, BMEnewHumValue, tempUnit, presUnit);
  } else
  {
    debugLn(F("BME280: Disabled due to init failure"));
    BMEnewTempValue = 99;
    BMEnewHumValue  = 99;
    BMEnewPresValue = 39;
  }
  // check temp difference - update the status?
  if (checkBoundSensor(BMEnewTempValue, BMEtempValue, BMEdiffTemp)) {
    BMEtempValue = BMEnewTempValue;
    dtostrf(BMEtempValue, 3, 1, sBMEtempValue);
    debugLn(F("BME280: Publish new Temp via MQTT"));
    mqttClient.publish(bmeTempTopic, mqttQOSLevel, false, String(sBMEtempValue).c_str());      
  }
  // check humidity difference - do we need to update the status
  if (checkBoundSensor(BMEnewHumValue, BMEhumValue, BMEdiffHum)) {
    BMEhumValue = BMEnewHumValue;
    dtostrf(BMEhumValue, 3, 1, sBMEhumValue);
    debugLn(F("BME280: Publish new Humi via MQTT"));
    mqttClient.publish(bmeHumiTopic, mqttQOSLevel, false, String(sBMEhumValue).c_str());   
  }
  if (checkBoundSensor(BMEnewPresValue, BMEpresValue, BMEdiffPres)) {
    BMEpresValue = BMEnewPresValue;
    dtostrf(BMEpresValue, 3, 3, sBMEpresValue);
    debugLn(F("BME280: Publish new Pres via MQTT"));
    mqttClient.publish(bmePresTopic, mqttQOSLevel, false, String(sBMEpresValue).c_str());   
  }
  float BMEnewfeelValue = EnvironmentCalculations::HeatIndex(BMEnewTempValue, BMEnewHumValue, envTempUnit);
  if (checkBoundSensor(BMEnewfeelValue, BMEfeelValue, BMEdiffFeel)) {
    BMEfeelValue = BMEnewfeelValue;
    dtostrf(BMEfeelValue, 3, 1, sBMEfeelValue);
    debugLn(F("BME280: Publish new Feel via MQTT"));
    mqttClient.publish(bmeFeelTopic, mqttQOSLevel, false, String(sBMEfeelValue).c_str());   
  } 
  debugLn(String(F("BME280: Results - Temp: "))+ sBMEtempValue + F(" Humi: ") + sBMEhumValue + F(" Pres: ") + sBMEpresValue + F(" Feel: ") + sBMEfeelValue);
}  

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// onMqttConnect                                                 //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void onMqttConnect(bool sessionPresent) {
  debugLn(F("MQTT: Connected"));
  mqttClient.publish(lwtTopic, 2, true, mqttBirth);
  // Setup Sensor Polling
  bme280Tick.attach(bmePoll, checkBME280);
  rePushTick.attach(rePushPoll, rePushVals);
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// onMqttDisconnect                                              //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  debugLn(F("MQTT: Disconnected."));
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
  // if MQTT is disconnected stop polling sensors
  bme280Tick.detach();
  rePushTick.detach();
}

void flipLED1() {
  digitalWrite(intLED1Pin, !(digitalRead(intLED1Pin)));  //Invert Current State of LED  
}

const char *getDeviceID() {
  char *identifier = new char[30];
  os_strcpy(identifier, hostName);
  strcat_P(identifier, PSTR("-"));

  char cidBuf[7];
  sprintf(cidBuf, "%06X", ESP.getChipId());
  os_strcat(identifier, cidBuf);

  return identifier;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// ESP Setup                                                     //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void setup() {
  #ifdef DEBUGSERIAL
    Serial.begin(115200);
    while(!Serial) {} // Wait
    Serial.println();
  #endif  
  debugLn(String(F("digiVent - Build: ")) + F(__DATE__) + " " +  F(__TIME__));
  // build hostname with last 6 of MACID
  os_strcpy(mcuHostName, getDeviceID());

  // ~~~~ Set MQTT Topics
  sprintf_P(lwtTopic, PSTR("%s/LWT"), mcuHostName);
  sprintf_P(pirTopic, PSTR("%s/PIR"), mcuHostName);
  sprintf_P(rssiTopic, PSTR("%s/RSSI"), mcuHostName);
  sprintf_P(buildTopic, PSTR("%s/BUILD"), mcuHostName);
  sprintf_P(bmeTempTopic, PSTR("%s/BME280/Temp"), mcuHostName);
  sprintf_P(bmeHumiTopic, PSTR("%s/BME280/Humi"), mcuHostName);
  sprintf_P(bmePresTopic, PSTR("%s/BME280/Pres"), mcuHostName);
  sprintf_P(bmeFeelTopic, PSTR("%s/BME280/Feel"), mcuHostName);
 
  // ~~~~ Set PIN Modes
  Wire.begin(sdaPin, sclPin);
  pinMode(intLED1Pin,OUTPUT); 
  pinMode(intLED2Pin,OUTPUT);
  pinMode(pirPin,     INPUT);  // no pull-up with AM312

  // change to 0.5 second LED1 blink during sensor setup
  digitalWrite(intLED1Pin, LEDoff);
  digitalWrite(intLED2Pin, LEDoff);
  delay(10);  
  
  led1FlipTick.attach(0.4, flipLED1);
  while(!bme.begin())
  {
    debugLn(F("BME280: Not found"));
    delay(1000);
  }
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       debugLn(F("BME280: Init Completed"));
       bme280Avail = true;
       break;
     case BME280::ChipModel_BMP280:
       debugLn(F("BME280: BMP280 Detected. Meh...Disabling BME Reads"));
       bme280Avail = false;
       break;
     default:
       debugLn(F("BME280: Init failure. Meh...Disabling BME Reads"));
       bme280Avail = false;
  }
  led1FlipTick.detach();
  digitalWrite(intLED1Pin, LEDoff);
  digitalWrite(intLED2Pin, LEDoff);
  delay(10);  

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  // setup MQTT
  mqttClient.setWill(lwtTopic,2,true,mqttDeath,0);
  mqttClient.setCredentials(mqttUser,mqttPass);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setMaxTopicLength(512);
  mqttClient.setClientId(mcuHostName);

  connectToWifi();

  #ifdef DEBUGTELNET
    // Setup telnet server for remote debug output
    telnetServer.setNoDelay(true);
    telnetServer.begin();
    debugLn(String(F("Telnet: Started on port 23 - IP:")) + WiFi.localIP().toString());
  #endif

  // OTA Flash Sets
  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setHostname(mcuHostName);
  ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  // Setup HTTP Flash Page
  httpUpdater.setup(&httpServer, "/flash", update_username, update_password);
  httpServer.on("/restart", []() {
    debugLn(F("HTTP: Restart request received."));
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send(200, "text/plain", "Restart command sent to ESP Chip..." );
    delay(100);
    ESP.restart();
  });
  
  httpServer.begin();
  debugLn(F("ESP: Boot completed - Starting loop"));
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// rePushVals                                                    //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void rePushVals() {
    debugLn(F("ESP: rePushingVals to MQTT"));
    mqttClient.publish(bmeTempTopic, mqttQOSLevel, false, String(sBMEtempValue).c_str());      
    mqttClient.publish(bmeHumiTopic, mqttQOSLevel, false, String(sBMEhumValue).c_str());   
    mqttClient.publish(bmePresTopic, mqttQOSLevel, false, String(sBMEpresValue).c_str());   
    mqttClient.publish(bmeFeelTopic, mqttQOSLevel, false, String(sBMEfeelValue).c_str()); 
    mqttClient.publish(rssiTopic, mqttQOSLevel, false, String(WiFi.RSSI()).c_str()); 
    mqttClient.publish(buildTopic, mqttQOSLevel, false, String(String(F("digiVent - Build: ")) + F(__DATE__) + " " +  F(__TIME__) + " - " + WiFi.localIP().toString()).c_str()); 
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// checkPIR                                                      //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void checkPIR() {
  pirValue = digitalRead(pirPin); //read state of the PIR
  // AM312 goes LOW with no motion
  if (pirValue == LOW && pirStatus != 0) {
    debugLn(F("PIR: Motion Cleared - Publish to MQTT"));
    mqttClient.publish(pirTopic, mqttQOSLevel, false, pirOffStr);
    pirStatus = 0;
  }
  else if (pirValue == HIGH && pirStatus != 1) {
    digitalWrite(intLED1Pin, LEDon);
    debugLn(F("PIR: Motion Detected - Publish to MQTT"));
    mqttClient.publish(pirTopic, mqttQOSLevel, false, pirOnStr);
    pirStatus = 1;
  }
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Telnet                                                        //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
#ifdef DEBUGTELNET
void handleTelnetClient()
{ 
  if (telnetServer.hasClient())
  {
    // client is connected
    if (!telnetClient || !telnetClient.connected())
    {
      if (telnetClient)
        telnetClient.stop();                   // client disconnected
      telnetClient = telnetServer.available(); // ready for new client
    }
    else
    {
      telnetServer.available().stop(); // have client, block new connections
    }
  }
  // Handle client input from telnet connection.
  if (telnetClient && telnetClient.connected() && telnetClient.available())
  {
    // client input processing
    while (telnetClient.available())
    {
      // Read data from telnet just to clear out the buffer
      telnetClient.read();
    }
  }
}
#endif

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// Serial and Telnet Log Handler                                 //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void debugLn(String debugText)
{ 
  String debugTimeText = "[+" + String(float(millis()) / 1000, 3) + "s] " + debugText;
  #ifdef DEBUGSERIAL
    Serial.println(debugTimeText);
    Serial.flush();
  #endif
  #ifdef DEBUGTELNET
    if (telnetClient.connected())
    {
      debugTimeText += "\r\n";
      const size_t len = debugTimeText.length();
      const char *buffer = debugTimeText.c_str();
      telnetClient.write(buffer, len);
      handleTelnetClient();
    }
  #endif
}
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
// ESP Loop                                                      //
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= //
void loop() {
  ArduinoOTA.handle();
  httpServer.handleClient();
  #ifdef DEBUGTELNET
    handleTelnetClient();
  #endif
  checkPIR();
  if (initBoot) {   // on first loop pull sensors
    delay(2000);  // hold up before first pull, had exception issues in mqtt connect on early pulls
    initBoot = false;
    checkBME280();
    delay(20);
    if (WiFi.isConnected() && mqttClient.connected()) {
      mqttClient.publish(rssiTopic, mqttQOSLevel, false, String(WiFi.RSSI()).c_str()); 
      mqttClient.publish(buildTopic, mqttQOSLevel, false, String(String(F("digiVent - Build: ")) + F(__DATE__) + " " +  F(__TIME__) + " - " + WiFi.localIP().toString()).c_str()); 
      debugLn(String(F("digiVent - Build: ")) + F(__DATE__) + " " +  F(__TIME__) + " - " + WiFi.localIP().toString());
    } 
  }
} 

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "max6675.h"

// debug options

#define STA false

// defines

#define BAG_1 13
#define BAG_2 12
#define BAG_3 14
#define BAG_4 27

// MAX6675 configs

int thermoDO = 12;
int thermoCS = 15;
int thermoCLK = 14;

float Temperature;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// wifi configs

const char* ssid = "MD-4000SX";
const char* password = "12345678";

const char *soft_ap_ssid          = "Aldeidos";
const char *soft_ap_password      = NULL; // NULL for no password
const int   channel        = 10;    // WiFi Channel number between 1 and 13
const bool  hide_SSID      = false; // To disable SSID broadcast -> SSID will not appear in a basic WiFi scan
const int   max_connection = 2;     // Maximum simultaneous connected clients on the AP

// Set your Static IP address
IPAddress local_IP(192, 168, 2, 101);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

AsyncWebServer server(80);

AsyncWebSocket ws("/ws");

String message = "";
String Speed = "";
String position;

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;

String output = "";

double now, count = millis();

void Plot_Values(){
  // String Message = sentido+"&"+String(encoder_value)+"&"+String(Speed)+"&"+ActualGear;
  
  JsonDocument MessageJson;

  MessageJson["Fase1"] = !digitalRead(BAG_1) ? 1:0;
  MessageJson["Fase2"] = !digitalRead(BAG_2) ? 1:0;
  MessageJson["Fase3"] = !digitalRead(BAG_3) ? 1:0;
  MessageJson["Fase4"] = !digitalRead(BAG_4) ? 1:0;
  MessageJson["Volt1"] = volts0;
  MessageJson["Volt2"] = volts1;
  MessageJson["Volt3"] = volts2;
  MessageJson["Volt4"] = volts3;
  MessageJson["Temperature"] = Temperature;

  serializeJson(MessageJson, output);
  ws.textAll(output);

  Serial.println(output);

}

void initWiFi() {
  WiFi.mode(WIFI_AP_STA);

  Serial.println("\n[*] Creating ESP32 AP");
  WiFi.softAP(soft_ap_ssid, soft_ap_password, channel, hide_SSID, max_connection);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  if (STA){

    // Configures static IP address
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
    }

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
    Serial.println(WiFi.localIP());
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    // position = message.substring(0, message.indexOf("&"));
    // Serial.print("Position: ");Serial.println(position);
    // Speed = message.substring(message.indexOf("&")+1, message.length()).toInt();
    // Serial.print("Speed: ");Serial.println(Speed);
    Plot_Values();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      //Notify client of motor current state when it first connects
      Plot_Values();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
     break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup(void){
  
  Serial.begin(115200);

  pinMode(BAG_1, INPUT);
  pinMode(BAG_2, INPUT_PULLUP);
  pinMode(BAG_3, INPUT);
  pinMode(BAG_4, INPUT);


  initWiFi();
  initWebSocket();

  server.begin();

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
 
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}
 
void loop(void){
  
  now = millis();

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
 
  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);
  
  Temperature = thermocouple.readCelsius();

  ws.cleanupClients();

  if(now - count > 1000){
    Plot_Values();
    count = millis();
  }

}
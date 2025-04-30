//IAP 2 20250407 Update Losant Certificate
/**************
 * ESP32 version 2.0.5 Arduino IDE 1.8.13
 * Front Motor - Motor 1
 * Back Motor - Motor 2
 * Side Motor - Motor 3(left), Motor 4(right)
 * Back Fans - FAN1(left), Fan2(right)
 * Side Fans - Fan3(left), Fan4(right)
 */

#include <Wire.h>
#include "esp_system.h"
#include <HardwareSerial.h>
#include "NimBLEDevice.h"
#include "DHT.h"
#include "Adafruit_CCS811.h"
#include "Adafruit_HTU21DF.h"
#include "Arduino.h"
#include "PCF8575.h"
#include <pca9633.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Time.h>
#include <TimeLib.h>
#include <WiFiClientSecure.h>
#include <Losant.h>
//#include <WiFi.h>           //New add 20250407

//define
HardwareSerial MySerial1(1);
#define DHTPIN 25
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE, 30);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_CCS811 ccs;
Adafruit_CCS811 ccs2;
PCA9633 rgbw;
PCF8575 pcf8575(0x26);
#define motor_PUL 4
#define motor1_ENA P16
#define motor1_DIR P17
#define motor2_ENA P14
#define motor2_DIR P15
#define motor3_ENA P12
#define motor3_DIR P13
#define motor4_ENA P10
#define motor4_DIR P11
#define relay1 P07
#define relay2 P06
#define relay3 P05
#define relay4 P04
#define relay5 P03
#define INH P02
#define ADDA P01
#define ADDB P00
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;

//variable
bool deviceConnected = false;
bool receivedFlag = false;
bool workingmoderunning = false;
char ble_receivedByte[20] = {};
int testmode = 1; //1 is testmode, 0 is working mode
int regenerationmode = 1; // 1 is regeration mode, 0 is absorption mode
int timer0_Flag = 0; //check timer triggered?
int timer1_Flag = 1; //check timer triggered?
int timer2_Flag = 0; 
int setTimer2 = 0;
int AbsorptionTime = 600; //seconds
int RegenerationTime = 3600; //seconds
int datalogTime = 10; //seconds
int relayTime = 60; //seconds
int cnt = 2;
int relaycase = 0;
float temp1 = 0;
float temp2 = 0;
float humid1 = 0;
float humid2 = 0;
int co2_1 = 0;
int co2_2 = 0;
int co2_in,co2_in2 = 0;
int co2_ext1,co2_ext2,co2_ext3,co2_ext4 = 0;

//function
void PCF8575_Setup();
void RELAY_Call(int relay, int start);

// WiFi credentials.
const char* WIFI_SSID = "CAGFM";
const char* WIFI_PASS = "CAGFM123";

// Losant credentials.
const char* LOSANT_DEVICE_ID = "6284ad69be8b58339f88092e";
const char* LOSANT_ACCESS_KEY = "267d1944-68b9-41fb-8032-6afdb0333a31";
const char* LOSANT_ACCESS_SECRET = "70d8425010795909e2934867ee7a9b2182f6bc6df529299f70a6df5cb38a1ec0";

const int BUTTON_PIN = 14;
const int LED_PIN = 12;

bool ledState = false;

//Updated Certificate 20250407
const char* rootCABuff = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n" \
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n" \
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n" \
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n" \
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n" \
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n" \
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n" \
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n" \
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n" \
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n" \
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n" \
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n" \
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n" \
"MrY=\n" \
"-----END CERTIFICATE-----\n";

// initiate the the wifi client
WiFiClientSecure wifiClient;

LosantDevice device(LOSANT_DEVICE_ID);

// Called whenever the device receives a command from the Losant platform.
void handleCommand(LosantCommand *command) {
  Serial.print("Command received: ");
  Serial.println(command->name);
}

void connect() {
  // Connect to Wifi.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiClient.setCACert(rootCABuff);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }  

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to Losant.
  Serial.println();
  Serial.print("Connecting to Losant...");

  device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);

  while(!device.connected()) {
    delay(500);
    Serial.print(".");
    device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
  }

  Serial.println("Connected!");
}


void dataReady() {
  Serial.println("Data Ready to Send!");

  // Losant uses a JSON protocol. Construct the simple state object.
  // { "button" : true }
  StaticJsonDocument<500> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  root["temp"] = temp1;
  root["humd"] = humid1;
  root["temp2"] = temp2;
  root["humd2"] = humid2;
  //root["CO2_1"] = co2_1;
  //root["CO2_2"] = co2_2;
  root["co2_in"] = co2_in;
  root["co2_in2"] = co2_in2;
  root["co2_ext1"] = co2_ext1;
  root["co2_ext2"] = co2_ext2; 
  root["co2_ext3"] = co2_ext3; 
  root["co2_ext4"] = co2_ext4; 
  // Send the state to Losant.
  device.sendState(root);
}

//setup
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 NITTO 2 PROGRAM START");
    MySerial1.begin(9600, SERIAL_8N1, 13, 14); //RX, TX, SubG
    PCF8575_Setup(); //IO_Control
    device.onCommand(&handleCommand);
    connect();
}

//loop
void loop() {
  if(MySerial1.available()){
    char c = MySerial1.read();
    if(c == 'R'){
      char command = MySerial1.read();
      if(command == '1'){
        Serial.println("R1");
        MySerial1.print("R1");
        RELAY_Call(relay3,1);
      }
      if(command == '2'){
        Serial.println("R2");
        MySerial1.print("R2");
        RELAY_Call(relay3,0);
      }
      if(command == '3'){
        Serial.println("R3");
        MySerial1.print("R3");
        RELAY_Call(relay4,1);
      }
      if(command == '4'){
        Serial.println("R4");
        MySerial1.print("R4");
        RELAY_Call(relay4,0);
      }
      if(command == '5'){ //extra fan
        MySerial1.print("R5");
        RELAY_Call(relay5,1);
      }
      if(command == '6'){ //extra fan
        MySerial1.print("R6");
        RELAY_Call(relay5,0);
      }
    }
    if(c == 'D'){
      String str = MySerial1.readString();
      //Serial.print("str:");Serial.println(str);
      String strs[10];
      int StringCount = 0;
      // Split the string into substrings
      while (str.length() > 0)
      {
        int index = str.indexOf(',');
        if (index == -1) // No space found
        {
          strs[StringCount++] = str;
          break;
        }
        else
        {
          strs[StringCount++] = str.substring(0, index);
          str = str.substring(index+1);
        }
      }
      temp1 = strs[0].toFloat();
      humid1 = strs[1].toFloat();
      temp2 = strs[2].toFloat();
      humid2 = strs[3].toFloat();
      //co2_1 = strs[4].toInt();
      //co2_2 = strs[5].toInt();
      co2_in = strs[4].toInt();
      co2_in2 = strs[5].toInt();
      co2_ext1 = strs[6].toInt();
      co2_ext2 = strs[7].toInt();
      co2_ext3 = strs[8].toInt();
      co2_ext4 = strs[9].toInt();
      //Serial.print("temp1:");Serial.println(temp1);
      //Serial.print("humid1:");Serial.println(humid1);
      //Serial.print("temp2:");Serial.println(temp2);
      //Serial.print("humid2:");Serial.println(humid2);
      //Serial.print("co2_1:");Serial.println(co2_1);
      //Serial.print("co2_2:");Serial.println(co2_2);
      dataReady(); 
    }
  }
  
  bool toReconnect = false;

  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected from WiFi");
    toReconnect = true;
  }

  if(!device.connected()) {
    Serial.println("Disconnected from Losant");
    toReconnect = true;
  }

  if(toReconnect) {
    connect();
  }

  device.loop();
}

void PCF8575_Setup(){
  Serial.println("PCF8575(MOTOR,RLY,TACH) Setup Function");
  pinMode(motor_PUL,OUTPUT);
  pcf8575.pinMode(motor1_ENA,OUTPUT);
  pcf8575.pinMode(motor1_DIR,OUTPUT);
  pcf8575.pinMode(motor2_ENA,OUTPUT);
  pcf8575.pinMode(motor2_DIR,OUTPUT);
  pcf8575.pinMode(motor3_ENA,OUTPUT);
  pcf8575.pinMode(motor3_DIR,OUTPUT);
  pcf8575.pinMode(motor4_ENA,OUTPUT);
  pcf8575.pinMode(motor4_DIR,OUTPUT);
  pcf8575.pinMode(relay1,OUTPUT);
  pcf8575.pinMode(relay2,OUTPUT);
  pcf8575.pinMode(relay3,OUTPUT);
  pcf8575.pinMode(relay4,OUTPUT);
  pcf8575.pinMode(relay5,OUTPUT);
  pcf8575.pinMode(INH,OUTPUT);
  pcf8575.pinMode(ADDA,OUTPUT);
  pcf8575.pinMode(ADDB,OUTPUT);
  pcf8575.begin();

  digitalWrite(motor_PUL,LOW);
  pcf8575.digitalWrite(motor1_ENA,LOW);
  pcf8575.digitalWrite(motor1_DIR,LOW);
  pcf8575.digitalWrite(motor2_ENA,LOW);
  pcf8575.digitalWrite(motor2_DIR,LOW);
  pcf8575.digitalWrite(motor3_ENA,LOW);
  pcf8575.digitalWrite(motor3_DIR,LOW);
  pcf8575.digitalWrite(motor4_ENA,LOW);
  pcf8575.digitalWrite(motor4_DIR,LOW);
  pcf8575.digitalWrite(relay1,LOW);
  pcf8575.digitalWrite(relay2,LOW);
  pcf8575.digitalWrite(relay3,LOW);
  pcf8575.digitalWrite(relay4,LOW);
  pcf8575.digitalWrite(relay5,LOW);
  pcf8575.digitalWrite(INH,LOW);
  pcf8575.digitalWrite(ADDA,LOW);
  pcf8575.digitalWrite(ADDB,LOW);

  delay(1000);
  Serial.println("PCF8575 Setup Done");
}

void RELAY_Call(int relay, int start){ //start:0, relay ON ; start:1, relay OFF
  if(start){
    Serial.println("RELAY ON");
    pcf8575.digitalWrite(relay,HIGH);
  } else{
    Serial.println("RELAY OFF");
    pcf8575.digitalWrite(relay,LOW);
  }
}
